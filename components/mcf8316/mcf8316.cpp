#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "mcf8316.h"

#include <algorithm>
#include <string>
#include <utility>

namespace esphome {
namespace mcf8316 {

const char *const TAG = "MCF8316";

#define RETURN_ERROR_IF_FAILED_OR_ASLEEP \
  { \
    if (this->is_failed()) return ErrorCode::ERROR_FAILED; \
    if (!this->awake_) return ErrorCode::ERROR_ASLEEP; \
  }

// 24-bit control word flags
constexpr uint32_t CTRL_OP_READ = 0x800000;
constexpr uint32_t CTRL_CRC_EN = 0x400000;
constexpr uint32_t CTRL_DLEN_32 = 0x100000;

struct CRC8 {
  uint8_t value = 0xff;

  void append(uint8_t data) {
    uint8_t crc = this->value ^ data;
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x80) {
        crc <<= 1;
        crc ^= 0x07;
      } else {
        crc <<= 1;
      }
    }
    this->value = crc;
  }

  void append(const uint8_t* data, size_t length) {
    while (length--) {
      this->append(*data++);
    }
  }
};

const char* ERROR_NAMES[] = {
  "NO_ERROR", "ERROR_I2C", "ERROR_CRC", "ERROR_DEVICE", "ERROR_ASLEEP", "ERROR_FAILED",
};

const char* MCF8316Component::error_name(ErrorCode code) {
  if (unsigned(code) < sizeof(ERROR_NAMES) / sizeof(ERROR_NAMES[0])) {
    return ERROR_NAMES[size_t(code)];
  }
  return "UNKNOWN";
}

float MCF8316Component::get_setup_priority() const { return setup_priority::HARDWARE; }

void MCF8316Component::dump_config() {
  ESP_LOGCONFIG(TAG, "MCF8316 component:");
  ESP_LOGCONFIG(TAG, "  Address: 0x%02X", this->address_);
  ESP_LOGCONFIG(TAG, "  WAKE Pin: %s", this->wake_pin_->dump_summary().c_str());
  ESP_LOGCONFIG(TAG, "  NFAULT Pin: %s", this->nfault_pin_->dump_summary().c_str());
  ESP_LOGCONFIG(TAG, "  Watchdog enabled: %d", this->watchdog_);

  log_config(this->config_shadow_);
}

Config MCF8316Component::make_default_config() const {
  Config config{};
  config.set(I2C_TARGET_ADDR, this->address_);
  config.set(SPEED_MODE, SpeedMode::DIGITAL_SPEED_CTRL); // set speed over I2C
  config.set(INPUT_REFERENCE_MODE, uint8_t(0)); // input reference is speed percentage
  config.set(BRAKE_INPUT, BrakeInput::OVERRIDE_OFF); // set brake over I2C
  config.set(DIR_INPUT, DirInput::OVERRIDE_CLOCKWISE); // set direction over I2C
  config.set(DEV_MODE, uint8_t(1u)); // enable sleep when wake pin is low (if set to 0, will go into standby instead)
  config.set(SLEEP_ENTRY_TIME, uint8_t(2)); // Sleep entry time: sleep when speed pin low for 20 ms
  if (this->watchdog_) {
    config.set(EXT_WDT_EN, true); // enable watchdog timer
    config.set(EXT_WDT_CONFIG, uint8_t(1u)); // 2 second timeout over I2C
    config.set(EXT_WDT_INPUT_MODE, uint8_t(0u)); // watchdog tickle over I2C
    config.set(EXT_WDT_FAULT_MODE, uint8_t(1u)); // latch in Hi-Z when watchdog fault occurs
  } else {
    config.set(EXT_WDT_EN, false);
  }
  return config;
}

void MCF8316Component::setup() {
  this->wake_pin_->setup();
  this->nfault_pin_->setup();

  // We don't know whether the device has been programmed to use the SPEED/WAKE pin
  // exclusively for wake-ups.  Until we know for sure, we don't want to set the pin
  // high because it could start the motor running at full speed.  Try to load the
  // PIN_CONFIG register into the config shadow so that the wake/sleep functions can
  // check the SPEED_MODE parameter and do the right thing.
  this->wake_pin_->digital_write(false);
  this->awake_ = true; // pretend the device is already awake
  if (this->read_register_(Register::PIN_CONFIG,
      &this->config_shadow_.at<Register::PIN_CONFIG>().value, true /*silence_logs*/)) {
    // The device did not respond.  Assume it's asleep.
    this->config_shadow_.set(SPEED_MODE, SpeedMode::DIGITAL_SPEED_CTRL);
  }
  this->awake_ = false; // stop pretending

  // Now wake up for real, if needed.
  this->wake_();
  this->clear_fault();

  // This is important so retry just in case.
  ErrorCode error = this->load_config_from_eeprom();
  if (error) {
    delay(10);
    error = this->load_config_from_eeprom();
  }
  if (error) {
    this->mark_failed("Failed to load config from EEPROM");
  }
}

void MCF8316Component::loop() {
  if (this->awake_) {
    if (this->watchdog_) {
      this->tickle_watchdog_();
    }
    this->check_fault_();
    this->check_algorithm_state_();
  }
}

void MCF8316Component::wake_() {
  if (!this->awake_) {
    this->awake_ = true;
    this->update_wake_state_for_pin_config_();
    delay(5); // time to wake is 3 to 5 ms according to the datasheet
  }
}

void MCF8316Component::sleep_() {
  if (this->awake_) {
    this->awake_ = false;
    this->update_wake_state_for_pin_config_();
  }
}

bool MCF8316Component::is_wake_pin_configured_() const {
  return this->config_shadow_.get(SPEED_MODE) == SpeedMode::DIGITAL_SPEED_CTRL;
}

void MCF8316Component::update_wake_state_for_pin_config_() {
  this->wake_pin_->digital_write(this->awake_ && this->is_wake_pin_configured_());
}

void MCF8316Component::tickle_watchdog_() {
  constexpr uint32_t TICKLE_INTERVAL = 250; // timeout is 2000 ms
  uint32_t now = millis();
  if (now - this->last_tickle_time_ < TICKLE_INTERVAL) {
    return;
  }

  RegisterValue<Register::ALGO_CTRL1> algo_ctrl1;
  algo_ctrl1.set(WATCHDOG_TICKLE, true);
  ErrorCode error = this->write(algo_ctrl1);
  if (error) {
    if (!this->tickle_failure_count_) {
      ESP_LOGW(TAG, "Failed to tickle watchdog timer: %s", error_name(error));
      this->update_warning_();
    }
    constexpr uint8_t FAIL_LIMIT = 4;
    if (this->tickle_failure_count_++ >= FAIL_LIMIT) {
      this->tickle_failure_count_ = 1;
      this->mark_failed("Device not responding");
    }
  } else {
    if (this->tickle_failure_count_) {
      this->tickle_failure_count_ = 0;
      this->update_warning_();
    }
  }
  this->last_tickle_time_ = now;
}

void MCF8316Component::check_fault_() {
  if (this->nfault_pin_->digital_read()) {
    if (!this->fault_status_.is_faulted()) {
      return;
    }
    this->fault_status_ = {};
  } else {
    FaultStatus fault_status{};
    if (this->read_register_(Register::GATE_DRIVER_FAULT_STATUS, reinterpret_cast<uint32_t*>(&fault_status.gate_driver))
        || this->read_register_(Register::CONTROLLER_FAULT_STATUS, reinterpret_cast<uint32_t*>(&fault_status.controller))
        || fault_status == this->fault_status_) {
      return;
    }
    this->fault_status_ = fault_status;
    this->log_fault_();
  }
  this->update_warning_();
  this->on_fault_callback_.call(this->fault_status_);
}

void MCF8316Component::log_fault_() {
  if (this->fault_status_.gate_driver) {
    ESP_LOGW(TAG, "Gate driver fault: %s",
        format_gate_driver_fault_status(this->fault_status_.gate_driver).c_str());
  }
  if (this->fault_status_.controller) {
    ESP_LOGW(TAG, "Controller fault: %s",
        format_controller_fault_status(this->fault_status_.controller).c_str());
  }
}

void MCF8316Component::clear_fault() {
  RegisterValue<Register::ALGO_CTRL1> algo_ctrl1;
  algo_ctrl1.set(CLR_FLT, true);
  algo_ctrl1.set(CLR_FLT_RETRY_COUNT, true);
  ErrorCode error = this->write(algo_ctrl1);
  if (error) {
    ESP_LOGW(TAG, "Failed to clear faults: %s", error_name(error));
  }
}

void MCF8316Component::update_warning_() {
  if (this->fault_status_.is_faulted()) {
    this->status_set_warning("Fault reported");
  } else if (this->tickle_failure_count_) {
    this->status_set_warning("Failed to tickle watchdog timer");
  } else if (this->algorithm_state_failure_count_) {
    this->status_set_warning("Failed to read algorithm state");
  } else {
    this->status_clear_warning();
  }
}

void MCF8316Component::check_algorithm_state_() {
  RegisterValue<Register::ALGORITHM_STATE> algorithm_state_value;
  ErrorCode error = read(&algorithm_state_value);
  if (error) {
    if (!this->algorithm_state_failure_count_) {
      ESP_LOGW(TAG, "Failed to read algorithm state: %s", error_name(error));
      this->update_warning_();
    }
    constexpr uint8_t FAIL_LIMIT = 4;
    if (this->algorithm_state_failure_count_++ >= FAIL_LIMIT) {
      this->algorithm_state_failure_count_ = 1;
      this->mark_failed("Device not responding");
    }
    return; // skip it
  } else {
    if (this->algorithm_state_failure_count_) {
      this->algorithm_state_failure_count_ = 0;
      this->update_warning_();
    }
  }

  AlgorithmState algorithm_state = algorithm_state_value.get(ALGORITHM_STATE);
  if (algorithm_state != last_algorithm_state_) {
    last_algorithm_state_ = algorithm_state;
    ESP_LOGD(TAG, "Algorithm state: %s", algorithm_state_name(algorithm_state));
  }
  this->check_mpet_algorithm_state_(algorithm_state);
}

MCF8316Component::ErrorCode MCF8316Component::read_config() {
  RETURN_ERROR_IF_FAILED_OR_ASLEEP;

  ESP_LOGI(TAG, "Reading configuration shadow registers");
  for (size_t i = 0; i < Config::LENGTH; i++) {
    ErrorCode error = this->read_register_(Config::index_to_register(i),
        &this->config_shadow_.register_values[i].value);
    if (error) {
      ESP_LOGE(TAG, "Failed to read configuration shadow registers: %s", error_name(error));
      return error;
    }
  }
  this->update_wake_state_for_pin_config_();
  return ErrorCode::NO_ERROR;
}

MCF8316Component::ErrorCode MCF8316Component::write_config(Config config) {
  RETURN_ERROR_IF_FAILED_OR_ASLEEP;

  ESP_LOGI(TAG, "Writing configuration shadow registers");
  for (size_t i = 0; i < Config::LENGTH; i++) {
    ErrorCode error = this->modify_config_register_with_workarounds_(Config::index_to_register(i), config.register_values[i]);
    if (error) {
      ESP_LOGE(TAG, "Failed to write configuration shadow registers: %s", error_name(error));
      return error;
    }
  }
  return ErrorCode::NO_ERROR;
}

MCF8316Component::ErrorCode MCF8316Component::load_config_from_eeprom() {
  RETURN_ERROR_IF_FAILED_OR_ASLEEP;

  ESP_LOGI(TAG, "Loading configuration shadow registers from EEPROM");
  RegisterValue<Register::ALGO_CTRL1> algo_ctrl1;
  algo_ctrl1.set(EEPROM_READ, true);
  ErrorCode error = this->write(algo_ctrl1);
  if (error) {
    ESP_LOGE(TAG, "Failed to load configuration shadow registers from EEPROM: %s", error_name(error));
    return error;
  }

  // Wait 200 ms for the read to complete, according to SLLA664 application note
  delay(200);
  error = this->read(&algo_ctrl1);
  if (error || algo_ctrl1.value != 0) {
    ESP_LOGW(TAG, "Failed to confirm the configuration shadow registers were loaded from the EEPROM: %s", error_name(error));
    // Continue to try to read the configuration anyway
  }

  // Read the configuration
  error = this->read_config();
  if (error) {
    return error;
  }
  this->config_eeprom_ = this->config_shadow_;
  return ErrorCode::NO_ERROR;
}

MCF8316Component::ErrorCode MCF8316Component::save_config_to_eeprom() {
  RETURN_ERROR_IF_FAILED_OR_ASLEEP;

  if (this->config_shadow_.equals_ignoring_config_register_parity(this->config_eeprom_)) {
    ESP_LOGD(TAG, "Not saving configuration shadow registers to the EEPROM because they have not changed");
    return ErrorCode::NO_ERROR;
  }

  ESP_LOGI(TAG, "Saving configuration shadow registers to the EEPROM");

  RegisterValue<Register::ALGO_CTRL1> algo_ctrl1;
  algo_ctrl1.set(EEPROM_WRT, true);
  algo_ctrl1.set(EEPROM_WRITE_ACCESS_KEY, uint8_t(0xa5));
  ErrorCode error = this->write(algo_ctrl1);
  if (error) {
    ESP_LOGE(TAG, "Failed to save the configuration shadow registers to the EEPROM: %s", error_name(error));
    return error;
  }

  // Wait 750 ms for the write to complete, according to the datasheet
  // TODO: Should move this check to an interval timer
  delay(750);
  error = this->read(&algo_ctrl1);
  if (error || algo_ctrl1.value != 0) {
    ESP_LOGE(TAG, "Failed to confirm the configuration shadow registers were saved to the EEPROM: %s", error_name(error));
    return error ? error : ErrorCode::ERROR_DEVICE;
  }

  this->config_eeprom_ = this->config_shadow_;
  return ErrorCode::NO_ERROR;
}

MCF8316Component::ErrorCode MCF8316Component::start_mpet(bool write_shadow) {
  RETURN_ERROR_IF_FAILED_OR_ASLEEP;

  ESP_LOGI(TAG, "Starting motor parameter extraction tool");
  RegisterValue<Register::ALGO_DEBUG2> algo_debug2;
  algo_debug2.set(MPET_CMD, uint8_t(1u));
  algo_debug2.set(MPET_R, true);
  algo_debug2.set(MPET_L, true);
  algo_debug2.set(MPET_KE, true);
  algo_debug2.set(MPET_MECH, true);
  algo_debug2.set(MPET_WRITE_SHADOW, write_shadow);
  ErrorCode error = this->write(algo_debug2);
  if (error) {
      ESP_LOGE(TAG, "Failed to start MPET: %s", error);
      return error;
  }
  this->mpet_in_progress_ = true;
  this->mpet_may_write_shadow_ = write_shadow;
  this->mpet_start_time_ = millis();
  return ErrorCode::NO_ERROR;
}

void MCF8316Component::check_mpet_algorithm_state_(AlgorithmState algorithm_state) {
  constexpr uint32_t MPET_MINIMUM_RUNTIME_MS = 500;
  if (!this->mpet_in_progress_ || is_mpet_running(algorithm_state) ||
      millis() - this->mpet_start_time_ < MPET_MINIMUM_RUNTIME_MS) {
    return; // wait for MPET to finish
  }

  this->mpet_in_progress_ = false;
  ESP_LOGI(TAG, "Finished motor parameter extraction");
  ESP_LOGI(TAG, "Results:");

  RegisterValue<Register::ALGO_STATUS_MPET> algo_status_mpet;
  if (!read(&algo_status_mpet)) {
    ESP_LOGI(TAG, "  MPET_R_STATUS: %d", algo_status_mpet.get(MPET_R_STATUS));
    ESP_LOGI(TAG, "  MPET_L_STATUS: %d", algo_status_mpet.get(MPET_L_STATUS));
    ESP_LOGI(TAG, "  MPET_KE_STATUS: %d", algo_status_mpet.get(MPET_KE_STATUS));
    ESP_LOGI(TAG, "  MPET_MECH_STATUS: %d", algo_status_mpet.get(MPET_MECH_STATUS));
    ESP_LOGI(TAG, "  MPET_PWM_FREQ: %d", algo_status_mpet.get(MPET_PWM_FREQ));
  }
  RegisterValue<Register::MTR_PARAMS> mtr_params;
  if (!read(&mtr_params)) {
    ESP_LOGI(TAG, "  MOTOR_R: %d", mtr_params.get(MPET_MOTOR_R));
    ESP_LOGI(TAG, "  MOTOR_L: %d", mtr_params.get(MPET_MOTOR_L));
    ESP_LOGI(TAG, "  MOTOR_BEMF_CONST: %d", mtr_params.get(MPET_MOTOR_BEMF_CONST));
  }
  RegisterValue<Register::SPEED_PI> speed_pi;
  if (!read(&speed_pi)) {
    ESP_LOGI(TAG, "  SPEED_PI_LOOP_KI: %d", speed_pi.get(SPEED_PI_LOOP_KI));
    ESP_LOGI(TAG, "  SPEED_PI_LOOP_KP: %d", speed_pi.get(SPEED_PI_LOOP_KP));
  }
#if false // irrelevant because we evaluated a speed mode control loop
  RegisterValue<Register::CURRENT_PI> current_pi;
  if (!read(&current_pi)) {
    ESP_LOGI(TAG, "  CURRENT_PI_LOOP_KI: %d", current_pi.get(CURRENT_PI_LOOP_KI));
    ESP_LOGI(TAG, "  CURRENT_PI_LOOP_KP: %d", current_pi.get(CURRENT_PI_LOOP_KP));
  }
#endif

  if (this->mpet_may_write_shadow_) {
    this->mpet_may_write_shadow_ = false;
    if (!read_config()) {
      ESP_LOGI(TAG, "Motor configuration shadow registers after MPET:");
      ESP_LOGI(TAG, "  MOTOR_RES: %d", this->config_shadow_.get(MOTOR_RES));
      ESP_LOGI(TAG, "  MOTOR_IND: %d", this->config_shadow_.get(MOTOR_IND));
      ESP_LOGI(TAG, "  MOTOR_BEMF_CONST: %d", this->config_shadow_.get(MOTOR_BEMF_CONST));
      ESP_LOGI(TAG, "  SPD_LOOP_KP: %d", this->config_shadow_.get(SPD_LOOP_KP));
      ESP_LOGI(TAG, "  SPD_LOOP_KI: %d", this->config_shadow_.get(SPD_LOOP_KI));
#if false // irrelevant because we evaluated a speed mode control loop
      ESP_LOGI(TAG, "  CURR_LOOP_KP: %d", this->config_shadow_.get(CURR_LOOP_KP));
      ESP_LOGI(TAG, "  CURR_LOOP_KI: %d", this->config_shadow_.get(CURR_LOOP_KI));
#endif
    }
  }

  // Clear MPET flags so the tool doesn't run again right away if a fault is cleared
  RegisterValue<Register::ALGO_DEBUG2> algo_debug2;
  algo_debug2.set(MPET_CMD, uint8_t(0u));
  algo_debug2.set(MPET_R, false);
  algo_debug2.set(MPET_L, false);
  algo_debug2.set(MPET_KE, false);
  algo_debug2.set(MPET_MECH, false);
  algo_debug2.set(MPET_WRITE_SHADOW, false);
  this->write(algo_debug2);
}

MCF8316Component::ErrorCode MCF8316Component::read_speed_feedback(float* out_speed_in_rotor_hz) {
  RegisterValue<Register::SPEED_FDBK> speed_fdbk;
  ErrorCode error = read(&speed_fdbk);
  if (error) {
    return error;
  }
  const float speed_feedback = static_cast<int32_t>(speed_fdbk.value); // value is signed
  const float max_speed = this->config_shadow_.get(MAX_SPEED);
  const float speed_in_electrical_hz = speed_feedback * max_speed / (6 * (1 << 27));
  *out_speed_in_rotor_hz = this->convert_speed_in_electrical_hz_to_rotor_hz(speed_in_electrical_hz);
  return ErrorCode::NO_ERROR;
}

MCF8316Component::ErrorCode MCF8316Component::write_speed_input(float speed_in_rotor_hz) {
  const float max_speed = this->config_shadow_.get(MAX_SPEED);
  const float speed_in_electrical_hz = this->convert_speed_in_rotor_hz_to_electrical_hz(speed_in_rotor_hz);
  RegisterValue<Register::ALGO_DEBUG1> algo_debug1;
  algo_debug1.set(DIGITAL_SPEED_CTRL, uint16_t(std::clamp(speed_in_electrical_hz * (32768.f * 6) / max_speed, 0.f, 32767.f)));
  return this->write(algo_debug1);
}

float MCF8316Component::convert_speed_in_electrical_hz_to_rotor_hz(float speed_in_electrical_hz) const {
  const uint8_t fg_div = this->config_shadow_.get(FG_DIV);
  return mcf8316::convert_speed_in_electrical_hz_to_rotor_hz(speed_in_electrical_hz, fg_div);
}

float MCF8316Component::convert_speed_in_rotor_hz_to_electrical_hz(float speed_in_rotor_hz) const {
  const uint8_t fg_div = this->config_shadow_.get(FG_DIV);
  return mcf8316::convert_speed_in_rotor_hz_to_electrical_hz(speed_in_rotor_hz, fg_div);
}

MCF8316Component::ErrorCode MCF8316Component::write_brake_input_config(bool brake_on) {
  return this->modify_config_register<Register::PIN_CONFIG>([brake_on](RegisterValue<Register::PIN_CONFIG> value) {
    return value.set(BRAKE_INPUT, brake_on ? BrakeInput::OVERRIDE_ON : BrakeInput::OVERRIDE_OFF);
  });
}

MCF8316Component::ErrorCode MCF8316Component::write_direction_input_config(bool direction_counter_clockwise) {
  return this->modify_config_register<Register::PERI_CONFIG1>([direction_counter_clockwise](RegisterValue<Register::PERI_CONFIG1> value) {
    return value.set(DIR_INPUT, direction_counter_clockwise ? DirInput::OVERRIDE_COUNTER_CLOCKWISE : DirInput::OVERRIDE_CLOCKWISE);
  });
}

MCF8316Component::ErrorCode MCF8316Component::read_bus_current(float* out_current_in_amps) {
  RegisterValue<Register::BUS_CURRENT> bus_current;
  ErrorCode error = read(&bus_current);
  if (error) {
    return error;
  }
  *out_current_in_amps = float(static_cast<int32_t>(bus_current.value)) * 10 / (1 << 30);
  return ErrorCode::NO_ERROR;
}

MCF8316Component::ErrorCode MCF8316Component::read_motor_phase_peak_current(float* out_current_in_amps) {
  RegisterValue<Register::IMAG_SQR> imag_sqr;
  ErrorCode error = read(&imag_sqr);
  if (error) {
    return error;
  }
  *out_current_in_amps = std::sqrt(float(imag_sqr.value) / (1 << 27)) * 10;
  return ErrorCode::NO_ERROR;
}

MCF8316Component::ErrorCode MCF8316Component::read_vm_voltage(float* out_voltage_in_volts) {
  RegisterValue<Register::VM_VOLTAGE> vm_voltage;
  ErrorCode error = read(&vm_voltage);
  if (error) {
    return error;
  }
  *out_voltage_in_volts = float(vm_voltage.value) * 60 / (1 << 27);
  return ErrorCode::NO_ERROR;
}

MCF8316Component::ErrorCode MCF8316Component::modify_config_register_with_workarounds_(
    Register reg, RegisterValue_ register_value) {
  RegisterValue_& shadow_value = this->config_shadow_.register_values[Config::register_to_index(reg)];
  if (register_value.equals_ignoring_config_register_parity(shadow_value)) {
    // Store the updated parity bit so that the config shadow reflects what the caller last wrote
    // even though the device ignores the bit.
    shadow_value.value = register_value.value;
    return ErrorCode::NO_ERROR;
  }
  MCF8316Component::ErrorCode error;
  if (reg == Register::DEVICE_CONFIG2) {
    // The watchdog timer must be disabled before it is reconfigured, according to the datasheet
    auto shadow_value_typed = static_cast<const RegisterValue<Register::DEVICE_CONFIG2>&>(shadow_value);
    auto register_value_typed = static_cast<const RegisterValue<Register::DEVICE_CONFIG2>&>(register_value);
    if (register_value_typed.get(EXT_WDT_EN) && shadow_value_typed.get(EXT_WDT_EN) &&
        (register_value_typed.get(EXT_WDT_CONFIG) != shadow_value_typed.get(EXT_WDT_CONFIG) ||
            register_value_typed.get(EXT_WDT_INPUT_MODE) != shadow_value_typed.get(EXT_WDT_INPUT_MODE) ||
            register_value_typed.get(EXT_WDT_FAULT_MODE) != shadow_value_typed.get(EXT_WDT_FAULT_MODE))) {
      RegisterValue<Register::DEVICE_CONFIG2> modified_value_typed = shadow_value_typed;
      modified_value_typed.set(EXT_WDT_EN, false);
      ESP_LOGD(TAG, "Applying workaround for modification of watchdog configuration");
      if ((error = this->write_register_(reg, discard_config_register_parity(modified_value_typed.value)))) {
        return error;
      }
      shadow_value.value = modified_value_typed.value;
    }
  }
  if ((error = this->write_register_(reg, discard_config_register_parity(register_value.value)))) {
    return error;
  }
  shadow_value.value = register_value.value;
  if (reg == Register::PIN_CONFIG) {
    this->update_wake_state_for_pin_config_();
  }
  return ErrorCode::NO_ERROR;
}

MCF8316Component::ErrorCode MCF8316Component::read_register_(Register reg, uint32_t* out_value, bool silence_logs) {
  RETURN_ERROR_IF_FAILED_OR_ASLEEP;

  uint32_t ctrl = CTRL_OP_READ | CTRL_CRC_EN | CTRL_DLEN_32 | uint32_t(reg);
  uint8_t header[] = { uint8_t(ctrl >> 16), uint8_t(ctrl >> 8), uint8_t(ctrl) };
  // FIXME: Preferably this transaction would not assert a STOP condition on the I2C bus
  // between the write and the read.  It used to work in ESP-IDF 5.3.2 but broke in
  // ESP-IDF 5.4.2 which expects all transactions to end with a STOP.
  // See https://github.com/esphome/esphome/issues/10346
  esphome::i2c::ErrorCode i2c_error = this->bus_->write(this->address_, header, sizeof(header), true /*stop*/);
  if (i2c_error) {
    if (!silence_logs) {
      ESP_LOGE(TAG, "Failed to read register 0x%x: ERROR_I2C code %d", reg, i2c_error);
    }
    return ErrorCode::ERROR_I2C;
  }
  uint32_t data;
  uint8_t crc_actual;
  esphome::i2c::ReadBuffer read_buffers[] = {
    { reinterpret_cast<uint8_t*>(&data), sizeof(data) },
    { &crc_actual, 1 },
  };
  i2c_error = this->bus_->readv(this->address_, read_buffers, 2);
  if (i2c_error) {
    if (!silence_logs) {
      ESP_LOGE(TAG, "Failed to read register 0x%x: ERROR_I2C code %d", reg, i2c_error);
    }
    return ErrorCode::ERROR_I2C;
  }
  CRC8 crc_expected;
  crc_expected.append(this->address_ << 1);
  crc_expected.append(header, sizeof(header));
  crc_expected.append((this->address_ << 1) | 0x01);
  crc_expected.append(reinterpret_cast<uint8_t*>(&data), sizeof(data));
  if (crc_expected.value != crc_actual) {
    if (!silence_logs) {
      ESP_LOGE(TAG, "Failed to read register 0x%x: ERROR_CRC", reg);
    }
    return ErrorCode::ERROR_CRC;
  }
  *out_value = convert_little_endian(data);
  if (!silence_logs) {
    ESP_LOGVV(TAG, "Read register: 0x%x, value: 0x%08x", reg, *out_value);
  }
  return ErrorCode::NO_ERROR;
}

MCF8316Component::ErrorCode MCF8316Component::write_register_(Register reg, const uint32_t value, bool silence_logs) {
  RETURN_ERROR_IF_FAILED_OR_ASLEEP;

  if (!silence_logs) {
    ESP_LOGVV(TAG, "Write register: 0x%x, value: 0x%08x", reg, value);
  }
  uint32_t ctrl = CTRL_CRC_EN | CTRL_DLEN_32 | uint32_t(reg);
  uint8_t header[] = { uint8_t(ctrl >> 16), uint8_t(ctrl >> 8), uint8_t(ctrl) };
  uint32_t data = convert_little_endian(value);
  CRC8 crc;
  crc.append(this->address_ << 1);
  crc.append(header, sizeof(header));
  crc.append(reinterpret_cast<uint8_t*>(&data), sizeof(data));
  esphome::i2c::WriteBuffer write_buffers[] = {
    { header, sizeof(header) },
    { reinterpret_cast<uint8_t*>(&data), sizeof(data) },
    { &crc.value, 1 },
  };
  esphome::i2c::ErrorCode i2c_error = this->bus_->writev(this->address_, write_buffers, 3, true /*stop*/);
  if (i2c_error) {
    if (!silence_logs) {
      ESP_LOGE(TAG, "Failed to write register 0x%x: ERROR_I2C code %d", reg, i2c_error);
    }
    return ErrorCode::ERROR_I2C;
  }
  return ErrorCode::NO_ERROR;
}

}  // namespace mcf8316
}  // namespace esphome
