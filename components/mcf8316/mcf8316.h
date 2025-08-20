#pragma once

#include "registers.h"

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/components/i2c/i2c_bus.h"

#include <array>
#include <cstdint>

namespace esphome {
namespace mcf8316 {

// The MCF8316 BLDC motor controller driver.
//
// This component does not derive from I2CDevice because its register API is completely different.
class MCF8316Component : public Component {
 public:
  enum ErrorCode {
    NO_ERROR = 0,
    ERROR_I2C = 1, // I2C transaction failed
    ERROR_CRC = 2, // CRC checksum mismatch
    ERROR_DEVICE = 3, // Device reported an error during the operation
    ERROR_ASLEEP = 4, // Decide is asleep
    ERROR_FAILED = 5, // Component is in failed state and cannot be used
  };
  static const char* error_name(ErrorCode code);

  struct FaultStatus final {
    GateDriverFaultStatus gate_driver;
    ControllerFaultStatus controller;

    bool is_faulted() const { return gate_driver != 0 || controller != 0; }
    bool is_cleared() const { return !is_faulted(); }

    bool operator==(const FaultStatus&) const = default;
    bool operator!=(const FaultStatus&) const = default;
  };

  void set_i2c_address(uint8_t address) { this->address_ = address; }
  void set_i2c_bus(esphome::i2c::I2CBus *bus) { this->bus_ = bus; }

  void set_wake_pin(GPIOPin* pin) { this->wake_pin_ = pin; }
  void set_nfault_pin(GPIOPin* pin) { this->nfault_pin_ = pin; }
  void set_watchdog(bool enable) { this->watchdog_ = enable; }

  void add_on_fault_callback(std::function<void(FaultStatus)> &&callback) {
    this->on_fault_callback_.add(std::move(callback));
  }

  float get_setup_priority() const override;
  void setup() override;
  void loop() override;
  void dump_config() override;

  FaultStatus fault_status() const { return this->fault_status_; }
  bool is_faulted() const { return !this->fault_status_.is_faulted(); }
  void clear_fault();

  bool is_awake() const { return this->awake_; }
  void wake() { this->wake_(); }
  void sleep() { this->sleep_(); }

  // Returns a mostly zero-filled configuration with bits set for control over I2C.
  //
  // Configures the following settings:
  // - I2C_TARGET_ADDR: set as specified by set_i2c_address
  // - SPEED_MODE: digital speed control
  // - INPUT_REFERENCE_MODE: input reference is speed
  // - BRAKE_INPUT: override off
  // - DIR_INPUT: override clockwise
  // - DEV_MODE: enable sleep
  // - SLEEP_ENTRY_TIME: 200 us
  // - EXT_WDT_*: if watchdog_ is true, enable watchdog over I2C with 2 second timeout
  //
  // Note: These defaults are not sufficient to operate the motor.
  Config make_default_config() const;

  // Returns a cached copy of the configuration that was last loaded from or saved to EEPROM.
  // Represents the non-volatile configuration of the device.
  const Config& config_eeprom() const { return this->config_eeprom_; }

  // Returns a cached copy of the configuration that was last read from or written to the shadow registers.
  // Represents the volatile configuration of the device.
  const Config& config_shadow() const { return this->config_shadow_; }

  // Reads the configuration from the shadow registers.
  // Updates the contents of `config_shadow`.
  ErrorCode read_config();

  // Writes the provided configuration to the shadow registers.
  // Only writes registers whose contents differ from `config_shadow`.
  // Updates the contents of `config_shadow` with the changes.
  ErrorCode write_config(Config config);

  // Loads the configuration shadow registers from EEPROM as if the device was just powered on.
  // Updates the contents of `config_eeprom` and `config_shadow`.
  // Note: Must only be called when the motor is not spinning.
  ErrorCode load_config_from_eeprom();

  // Saves the configuration shadow registers to EEPROM from which they will be
  // automatically reloaded every time the device is powered on or wakes from sleep.
  // Does nothing if the contents of `config_shadow` and `config_eeprom` are the same.
  // Updates the contents of `config_eeprom` with the changes.
  // Note: Use this operation sparingly to avoid premature EEPROM wear.
  // Note: Must only be called when the motor is not spinning.
  ErrorCode save_config_to_eeprom();

  // Starts the motor parameter extraction tool.
  // If `write_shadow` is true, writes the motor parameters to the configuration
  // shadow registers upon successful completion.
  ErrorCode start_mpet(bool write_shadow);

  // Reads the actual motor speed in rotor Hz.
  // Applies `FG_DIV` to convert the speed feedback to rotor Hz.
  // The returned value is signed:
  // - Positive sign indicates movement in the clockwise direction.
  // - Negative sign indicates movement in the counter-clockwise direction.
  ErrorCode read_speed_feedback(float* out_speed_in_rotor_hz);

  // Writes the target motor speed in rotor Hz.
  // Applies `FG_DIV` to convert the speed input from rotor Hz.
  ErrorCode write_speed_input(float speed_in_rotor_hz);

  // Writes the brake input and updates `config_shadow`.
  ErrorCode write_brake_input_config(bool brake_on);

  // Writes the direction input and updates `config_shadow`.
  ErrorCode write_direction_input_config(bool direction_counter_clockwise);

  // Converts speed value from rotor Hz to electrical Hz based on `FG_DIV`.
  float convert_speed_in_electrical_hz_to_rotor_hz(float speed_in_electrical_hz) const;

  // Converts speed value from rotor Hz to electrical Hz based on `FG_DIV`.
  float convert_speed_in_rotor_hz_to_electrical_hz(float speed_in_rotor_hz) const;

  // Reads the BUS_CURRENT register and scales the result to amps.
  ErrorCode read_bus_current(float* out_current_in_amps);

  // Reads the IMAG_SQR register and scales the result to amps.
  ErrorCode read_motor_phase_peak_current(float* out_current_in_amps);

  // Reads the VM_VOLTAGE register and scales the result to volts.
  ErrorCode read_vm_voltage(float* out_voltage_in_volts);

  // Reads a non-configuration register from the device.
  template <Register reg>
  ErrorCode read(RegisterValue<reg>* out_register_value) {
    static_assert(!is_config_register(reg), "Use config_shadow() to access configuration registers.");
    return this->read_register_(reg, &out_register_value->value);
  }

  // Writes a register to the device.
  template <Register reg>
  ErrorCode write(RegisterValue<reg> register_value) {
    static_assert(!is_config_register(reg), "Use config_shadow() to access configuration registers.");
    return this->write_register_(reg, register_value.value);
  }

  // Modifies and writes a single config register and updates `config_shadow`.
  template <Register reg, typename Mutator>
  ErrorCode modify_config_register(Mutator mutator) {
    return this->modify_config_register_with_workarounds_(reg, mutator(this->config_shadow_.at<reg>()));
  }

 protected:
  void wake_();
  void sleep_();
  void tickle_watchdog_();
  void check_fault_();
  void log_fault_();
  void check_algorithm_state_();

  void update_warning_();

  bool is_wake_pin_configured_() const;
  void update_wake_state_for_pin_config_();
  void reset_with_sleep_cycle_();

  ErrorCode modify_config_register_with_workarounds_(Register reg, RegisterValue_ register_value);

  ErrorCode read_register_(Register reg, uint32_t* out_value, bool silence_logs = false);
  ErrorCode write_register_(Register reg, uint32_t value, bool silence_logs = false);

  Config config_eeprom_{}; ///< cached copy of non-volatile configuration
  Config config_shadow_{}; ///< cached copy of volatile configuration

  uint8_t address_{0x00};  ///< store the address of the device on the bus
  esphome::i2c::I2CBus *bus_{nullptr};   ///< pointer to I2CBus instance

  GPIOPin* wake_pin_{nullptr};
  GPIOPin* nfault_pin_{nullptr};
  bool watchdog_{};

  bool awake_{false};
  bool mpet_in_progress_{false};
  bool mpet_may_write_shadow_{false};
  AlgorithmState last_algorithm_state_{};
  uint8_t algorithm_state_failure_count_{};
  uint8_t tickle_failure_count_{};
  uint32_t last_tickle_time_{};

  FaultStatus fault_status_{};
  CallbackManager<void(FaultStatus)> on_fault_callback_;
};

}  // namespace mcf8316
}  // namespace esphome
