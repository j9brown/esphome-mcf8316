#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "registers.h"

#include <string>

namespace esphome {
namespace mcf8316 {

const char *const TAG = "MCF8316";

// Logs the contents of the configuration
void log_config_field_value(const char* name, uint32_t value) {
  ESP_LOGD(TAG, "      - %s: 0x%x", name, value);
}
#define MCF8316_LOG_CONFIG_FIELD(field) \
    log_config_field_value(#field, uint32_t(config.get(field)));

void log_config_register_value(const char* name, uint32_t value) {
  ESP_LOGD(TAG, "    %s: 0x%08x", name, value);
}
#define MCF8316_LOG_CONFIG_REGISTER(reg) \
    log_config_register_value(#reg, config.at<Register::reg>().value); \
    MCF8316_FOR_EACH_FIELD_OF_REGISTER(reg, MCF8316_LOG_CONFIG_FIELD);

void log_config(const Config& config) {
  ESP_LOGD(TAG, "  Configuration registers:");
  MCF8316_FOR_EACH_CONFIG_REGISTER(MCF8316_LOG_CONFIG_REGISTER)
}

bool is_mpet_running(AlgorithmState state) {
  return unsigned(state) >= unsigned(AlgorithmState::MOTOR_MPET_MOTOR_STOP_CHECK)
      && unsigned(state) < unsigned(AlgorithmState::MOTOR_MPET_DONE);
}

const char*  ALGORITHM_STATE_NAMES[] = {
  "IDLE", "ISD", "TRISTATE", "BRAKE_ON_START", "IPD", "SLOW_FIRST_CYCLE",
  "ALIGN", "OPEN_LOOP", "CLOSED_LOOP_UNALIGNED", "CLOSED_LOOP_ALIGNED",
  "CLOSED_LOOP_ACTIVE_BRAKING", "SOFT_STOP", "RECIRCULATE_STOP",
  "BRAKE_ON_STOP", "FAULT",
  "MPET_MOTOR_STOP_CHECK", "MPET_MOTOR_STOP_WAIT", "MPET_MOTOR_BRAKE",
  "MPET_ALGORITHM_PARAMETERS_INIT", "MPET_RL_MEASURE", "MPET_KE_MEASURE",
  "MPET_STALL_CURRENT_MEASURE", "MPET_TORQUE_MODE", "MPET_DONE", "MPET_FAULT",
};

const char* algorithm_state_name(AlgorithmState state) {
  if (unsigned(state) < sizeof(ALGORITHM_STATE_NAMES) / sizeof(ALGORITHM_STATE_NAMES[0])) {
    return ALGORITHM_STATE_NAMES[size_t(state)];
  }
  return "UNKNOWN";
}

const char* GATE_DRIVER_FAULT_BITS[32] = {
  "DRIVER_FAULT", nullptr, nullptr, "OCP",
  nullptr, "OVP", nullptr, nullptr,
  "OTW", "OTS", "OCP_HC", "OCP_LC",
  "OCP_HB", "OCP_LB", "OCP_HA", "OCP_LA",
  nullptr, nullptr, "BUCK_OCP", "BUCK_UV",
  "VCP_UV", nullptr, nullptr, nullptr,
  nullptr, nullptr, nullptr, nullptr,
  nullptr, nullptr, nullptr, nullptr,
};

const char* CONTROLLER_FAULT_BITS[32] = {
  "CONTROLLER_FAULT", nullptr, "IPD_FREQ_FAULT", "IPD_T1_FAULT",
  "IPD_T2_FAULT", nullptr, "MPET_IPD_FAULT", "MPET_BEMF_FAULT",
  "ABN_SPEED", "ABN_BEMF", "NO_MTR", "MTR_LCK",
  "LOCK_ILIMIT", "HW_LOCK_ILIMIT", "MTR_UNDER_VOLTAGE", "MTR_OVER_VOLTAGE",
  "SPEED_LOOP_SATURATION", "CURRENT_LOOP_SATURATION", "MAX_SPEED_SATURATION", "BUS_POWER_LIMIT_SATURATION",
  "EEPROM_WRITE_LOCK_SET", "EEPROM_READ_LOCK_SET", nullptr, nullptr,
  nullptr, "I2C_CRC_FAULT_STATUS", "EEPROM_ERR_STATUS", "BOOT_STL_FAULT",
  "WATCHDOG_FAULT", "CPU_RESET_FAULT_STATUS", "WWDT_FAULT_STATUS", nullptr,
};

std::string format_fault_bits(uint32_t value, const char* names[32]) {
  std::string result;
  if (value) {
    for (unsigned i = 0; i < 32; i++) {
      if (value & (0x80000000 >> i)) {
        if (!result.empty()) result.push_back('|');
        if (names[i]) {
          result.append(names[i]);
        } else {
          result.append(to_string(31 - i));
        }
      }
    }
  }
  return result;
}

std::string format_gate_driver_fault_status(uint32_t value) {
  return format_fault_bits(value, GATE_DRIVER_FAULT_BITS);
}

std::string format_controller_fault_status(uint32_t value) {
  return format_fault_bits(value, CONTROLLER_FAULT_BITS);
}

}  // namespace mcf8316
}  // namespace esphome
