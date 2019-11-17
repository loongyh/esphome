#include "esp32_touch.h"
#include "esphome/core/log.h"

#ifdef ARDUINO_ARCH_ESP32

namespace esphome {
namespace esp32_touch {

static const char *TAG = "esp32_touch";

void ESP32TouchComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ESP32 Touch Hub...");
  touch_pad_init();

  if (this->iir_filter_enabled_()) {
    touch_pad_filter_start(this->iir_filter_);
  }

  touch_pad_set_meas_time(this->sleep_cycle_, this->meas_cycle_);
  touch_pad_set_voltage(this->high_voltage_reference_, this->low_voltage_reference_, this->voltage_attenuation_);

  for (auto *child : this->children_) {
    // Disable interrupt threshold
    touch_pad_config(child->get_touch_pad(), 0);
    // Initialize adaptive threshold
    if (child->at_enabled()) {
      uint16_t value;
      if (this->iir_filter_enabled_()) {
        touch_pad_read_filtered(child->get_touch_pad(), &value);
      } else {
        touch_pad_read(child->get_touch_pad(), &value);
      }
      child->at_initialize(value);
    }
  }
}

void ESP32TouchComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Config for ESP32 Touch Hub:");
  ESP_LOGCONFIG(TAG, "  Meas cycle: %.2fms", this->meas_cycle_ / (8000000.0f / 1000.0f));
  ESP_LOGCONFIG(TAG, "  Sleep cycle: %.2fms", this->sleep_cycle_ / (150000.0f / 1000.0f));

  const char *lv_s;
  switch (this->low_voltage_reference_) {
    case TOUCH_LVOLT_0V5:
      lv_s = "0.5V";
      break;
    case TOUCH_LVOLT_0V6:
      lv_s = "0.6V";
      break;
    case TOUCH_LVOLT_0V7:
      lv_s = "0.7V";
      break;
    case TOUCH_LVOLT_0V8:
      lv_s = "0.8V";
      break;
    default:
      lv_s = "UNKNOWN";
      break;
  }
  ESP_LOGCONFIG(TAG, "  Low Voltage Reference: %s", lv_s);

  const char *hv_s;
  switch (this->high_voltage_reference_) {
    case TOUCH_HVOLT_2V4:
      hv_s = "2.4V";
      break;
    case TOUCH_HVOLT_2V5:
      hv_s = "2.5V";
      break;
    case TOUCH_HVOLT_2V6:
      hv_s = "2.6V";
      break;
    case TOUCH_HVOLT_2V7:
      hv_s = "2.7V";
      break;
    default:
      hv_s = "UNKNOWN";
      break;
  }
  ESP_LOGCONFIG(TAG, "  High Voltage Reference: %s", hv_s);

  const char *atten_s;
  switch (this->voltage_attenuation_) {
    case TOUCH_HVOLT_ATTEN_1V5:
      atten_s = "1.5V";
      break;
    case TOUCH_HVOLT_ATTEN_1V:
      atten_s = "1V";
      break;
    case TOUCH_HVOLT_ATTEN_0V5:
      atten_s = "0.5V";
      break;
    case TOUCH_HVOLT_ATTEN_0V:
      atten_s = "0V";
      break;
    default:
      atten_s = "UNKNOWN";
      break;
  }
  ESP_LOGCONFIG(TAG, "  Voltage Attenuation: %s", atten_s);

  if (this->iir_filter_enabled_()) {
    ESP_LOGCONFIG(TAG, "    IIR Filter: %ums", this->iir_filter_);
    touch_pad_filter_start(this->iir_filter_);
  } else {
    ESP_LOGCONFIG(TAG, "  IIR Filter DISABLED");
  }
  if (this->setup_mode_) {
    ESP_LOGCONFIG(TAG, "  Setup Mode ENABLED!");
  }

  for (auto *child : this->children_) {
    LOG_BINARY_SENSOR("  ", "Touch Pad", child);
    ESP_LOGCONFIG(TAG, "    Pad: T%d", child->get_touch_pad());
    ESP_LOGCONFIG(TAG, "    Threshold: %u", child->get_threshold());
    if (child->at_enabled()) {
      ESP_LOGCONFIG(TAG, "    Adaptive Threshold ENABLED!");
    }
  }
}

void ESP32TouchComponent::loop() {
  const uint32_t now = millis();
  bool should_print = this->setup_mode_ && now - this->setup_mode_last_log_print_ > 250;
  for (auto *child : this->children_) {
    uint16_t value;
    if (this->iir_filter_enabled_()) {
      touch_pad_read_filtered(child->get_touch_pad(), &value);
    } else {
      touch_pad_read(child->get_touch_pad(), &value);
    }

    child->value_ = value;
    child->publish_state(value < child->get_threshold());

    if (should_print) {
      ESP_LOGD(TAG, "Touch Pad '%s' (T%u): %u", child->get_name().c_str(), child->get_touch_pad(), value);
      if (child->at_enabled())
        ESP_LOGD(TAG, "Threshold: %u", child->get_threshold());
    }

    // Adaptive threshold 
    if (child->at_enabled()) {
      child->at_filter(now, value);
    }
  }

  if (should_print) {
    // Avoid spamming logs
    this->setup_mode_last_log_print_ = now;
  }
}

void ESP32TouchComponent::on_shutdown() {
  if (this->iir_filter_enabled_()) {
    touch_pad_filter_stop();
    touch_pad_filter_delete();
  }
  touch_pad_deinit();
}

ESP32TouchBinarySensor::ESP32TouchBinarySensor(const std::string &name, touch_pad_t touch_pad, uint16_t threshold)
    : BinarySensor(name), touch_pad_(touch_pad), threshold_(threshold) {}
ESP32TouchBinarySensor::ESP32TouchBinarySensor(const std::string &name, touch_pad_t touch_pad, bool at_enabled,
                                                uint16_t at_filter_period, float at_factor)
    : BinarySensor(name),
      touch_pad_(touch_pad),
      at_enabled_(at_enabled),
      at_filter_period_(at_filter_period),
      at_factor_(at_factor) {}

void ESP32TouchBinarySensor::at_initialize(uint16_t value) {
  float at_sample_period_ = (float)this->at_filter_period_ / 10.0;
  this->at_alpha_ = at_sample_period_ / (float)this->at_filter_period_;
  this->at_epsilon_m_ = (float)value * this->at_alpha_;
  this->at_epsilon_s_ = 0.1 * this->at_alpha_;
  this->at_median_ = (float)value;
}

void ESP32TouchBinarySensor::at_filter(uint32_t now, uint16_t value) {
  if (now - this->at_filter_last_run_ > this->at_filter_period_) {
    this->at_epsilon_m_ = this->at_median_ * this->at_alpha_;
    if (this->at_stddev_ < 0.1) {
      this->at_epsilon_s_ = 0.1;
    } else {
      this->at_epsilon_s_ = this->at_stddev_ * this->at_alpha_;
    }
    if (value > this->at_median_) {
      this->at_median_ += this->at_epsilon_m_;
    } else {
      this->at_median_ -= this->at_epsilon_m_;
    }
    uint16_t diff = value - this->at_median_;
    if (diff * diff > this->at_stddev_ * this->at_stddev_) {
      this->at_stddev_ += this->at_epsilon_s_;
    } else {
      this->at_stddev_ -= min(this->at_epsilon_s_, this->at_stddev_);
    }
    this->threshold_ = this->at_median_ - this->at_factor_ * this->at_stddev_;
    this->at_filter_last_run_ = now;
  }
}

}  // namespace esp32_touch
}  // namespace esphome

#endif
