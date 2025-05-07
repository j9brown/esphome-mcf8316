#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace esphome {
namespace mcf8316 {

enum class Register : uint32_t;

// Designates a bit-field of the specified width within a 32-bit register.
template <Register reg, unsigned bit_, unsigned width_, typename T>
struct Field final {
  using Type = T;
  static constexpr uint32_t mask = ((1ul << width_) - 1ul) << bit_;
  static constexpr unsigned bit = bit_;
  static constexpr unsigned width = width_;
};

// Designates a bit-field that has been split across two registers.
template <typename MSBField, typename LSBField, typename T>
struct CompositeField final {
  using Type = T;
  static constexpr MSBField msb = MSBField();
  static constexpr LSBField lsb = LSBField();
  static constexpr unsigned width = MSBField::width + LSBField::width;
};
template <typename T, typename MSBField, typename LSBField>
constexpr CompositeField<MSBField, LSBField, T> make_composite_field(MSBField msb, LSBField lsb) {
  return CompositeField<MSBField, LSBField, T>();
}

// MCF8316A register addresses (12 bits)
enum class Register : uint32_t {
  // Configuration shadow registers (restored from EEPROM on reset)
  ISD_CONFIG = 0x080,
  REV_DRIVE_CONFIG = 0x082,
  MOTOR_STARTUP1 = 0x084,
  MOTOR_STARTUP2 = 0x086,
  CLOSED_LOOP1 = 0x088,
  CLOSED_LOOP2 = 0x08a,
  CLOSED_LOOP3 = 0x08c,
  CLOSED_LOOP4 = 0x08e,
  FAULT_CONFIG1 = 0x090,
  FAULT_CONFIG2 = 0x092,
  REF_PROFILES1 = 0x094,
  REF_PROFILES2 = 0x096,
  REF_PROFILES3 = 0x098,
  REF_PROFILES4 = 0x09a,
  REF_PROFILES5 = 0x09c,
  REF_PROFILES6 = 0x09e,
  INT_ALGO1 = 0x0a0, // aka. INT_ALGO_1
  INT_ALGO2 = 0x0a2, // aka. INT_ALGO_2
  PIN_CONFIG = 0x0a4,
  DEVICE_CONFIG1 = 0x0a6,
  DEVICE_CONFIG2 = 0x0a8,
  PERI_CONFIG1 = 0x0aa,
  GD_CONFIG1 = 0x0ac,
  GD_CONFIG2 = 0x0ae,

  // Fault status registers
  GATE_DRIVER_FAULT_STATUS = 0x0e0,
  CONTROLLER_FAULT_STATUS = 0x0e2,
  EEPROM_FAULT_STATUS = 0x24c,

  // System status registers
  ALGO_STATUS = 0x0e4,
  MTR_PARAMS = 0x0e6,
  ALGO_STATUS_MPET = 0x0e8,

  // Algorithm control registers
  ALGO_CTRL1 = 0x0ea,
  ALGO_DEBUG1 = 0x0ec,
  ALGO_DEBUG2 = 0x0ee,
  CURRENT_PI = 0x0f0,
  SPEED_PI = 0x0f2,
  DAC_1 = 0x0f4,
  DAC_2 = 0x0f6,
  EEPROM_SECURITY = 0x0f8,

  // Algorithm variable registers
  ALGORITHM_STATE = 0x18e,
  FG_SPEED_FDBK = 0x194,
  BUS_CURRENT = 0x40c,
  PHASE_CURRENT_A = 0x444,
  PHASE_CURRENT_B = 0x446,
  PHASE_CURRENT_C = 0x448,
  IMAG_SQR = 0x46a,
  CSA_GAIN_FEEDBACK = 0x46c,
  VOLTAGE_GAIN_FEEDBACK = 0x477,
  VM_VOLTAGE = 0x47c,
  PHASE_VOLTAGE_VA = 0x484,
  PHASE_VOLTAGE_VB = 0x486,
  PHASE_VOLTAGE_VC = 0x488,
  SIN_COMMUTATION_ANGLE = 0x4bc,
  COS_COMMUTATION_ANGLE = 0x4be,
  IALPHA = 0x4dc,
  IBETA = 0x4de,
  VALPHA = 0x4e0,
  VBETA = 0x4e2,
  ID = 0x4ec,
  IQ = 0x4ee,
  VD = 0x4f0,
  VQ = 0x4f2,
  IQ_REF_ROTOR_ALIGN = 0x52a,
  SPEED_REF_OPEN_LOOP = 0x540,
  IQ_REF_OPEN_LOOP = 0x550,
  SPEED_REF_CLOSED_LOOP = 0x5d2,
  ID_REF_CLOSED_LOOP = 0x612,
  IQ_REF_CLOSED_LOOP = 0x614,
  ISD_STATE = 0x6ae,
  ISD_SPEED = 0x6b8,
  IPD_STATE = 0x6ea,
  IPD_ANGLE = 0x72e,
  ED = 0x772,
  EQ = 0x774,
  SPEED_FDBK = 0x782,
  THETA_EST = 0x786,
};

// Returns true if the register is a configuration shadow register.
constexpr inline bool is_config_register(Register reg) {
  return reg >= Register::ISD_CONFIG && reg <= Register::GD_CONFIG2;
}

#define MCF8316_FOR_EACH_CONFIG_REGISTER(x) \
  x(ISD_CONFIG) x(REV_DRIVE_CONFIG) x(MOTOR_STARTUP1) x(MOTOR_STARTUP2) \
  x(CLOSED_LOOP1) x(CLOSED_LOOP2) x(CLOSED_LOOP3) x(CLOSED_LOOP4) \
  x(FAULT_CONFIG1) x(FAULT_CONFIG2) x(REF_PROFILES1) x(REF_PROFILES2) \
  x(REF_PROFILES3) x(REF_PROFILES4) x(REF_PROFILES5) x(REF_PROFILES6) \
  x(INT_ALGO1) x(INT_ALGO2) x(PIN_CONFIG) x(DEVICE_CONFIG1) \
  x(DEVICE_CONFIG2) x(PERI_CONFIG1) x(GD_CONFIG1) x(GD_CONFIG2)

#define MCF8316_FOR_EACH_FIELD_OF_REGISTER(reg, x) MCF8316_FOR_EACH_ ## reg ## _FIELD(x)

// EEPROM register fields
// ISD_CONFIG
constexpr auto ISD_EN = Field<Register::ISD_CONFIG, 30, 1, bool>();
constexpr auto BRAKE_EN = Field<Register::ISD_CONFIG, 29, 1, bool>();
constexpr auto HIZ_EN = Field<Register::ISD_CONFIG, 28, 1, bool>();
constexpr auto RVS_DR_EN = Field<Register::ISD_CONFIG, 27, 1, bool>();
constexpr auto RESYNC_EN = Field<Register::ISD_CONFIG, 26, 1, bool>();
constexpr auto FW_DRV_RESYN_THR = Field<Register::ISD_CONFIG, 22, 4, uint8_t>();
constexpr auto BRK_MODE = Field<Register::ISD_CONFIG, 21, 1, uint8_t>();
constexpr auto BRK_CONFIG = Field<Register::ISD_CONFIG, 20, 1, uint8_t>();
constexpr auto BRK_CURR_THR = Field<Register::ISD_CONFIG, 17, 3, uint8_t>();
constexpr auto BRK_TIME = Field<Register::ISD_CONFIG, 13, 4, uint8_t>();
constexpr auto HIZ_TIME = Field<Register::ISD_CONFIG, 9, 4, uint8_t>();
constexpr auto STAT_DETECT_THR = Field<Register::ISD_CONFIG, 6, 3, uint8_t>();
constexpr auto REV_DRV_HANDOFF_THR = Field<Register::ISD_CONFIG, 2, 4, uint8_t>();
constexpr auto REV_DRV_OPEN_LOOP_CURRENT = Field<Register::ISD_CONFIG, 0, 2, uint8_t>();
#define MCF8316_FOR_EACH_ISD_CONFIG_FIELD(x) \
    x(ISD_EN) x(BRAKE_EN) x(HIZ_EN) x(RVS_DR_EN) x(RESYNC_EN) x(FW_DRV_RESYN_THR) \
    x(BRK_MODE) x(BRK_CONFIG) x(BRK_CURR_THR) x(BRK_TIME) x(HIZ_TIME) x(STAT_DETECT_THR) \
    x(REV_DRV_HANDOFF_THR) x(REV_DRV_OPEN_LOOP_CURRENT)

// REV_DRIVE_CONFIG
constexpr auto REV_DRV_OPEN_LOOP_ACCEL_A1 = Field<Register::REV_DRIVE_CONFIG, 27, 4, uint8_t>();
constexpr auto REV_DRV_OPEN_LOOP_ACCEL_A2 = Field<Register::REV_DRIVE_CONFIG, 23, 4, uint8_t>();
constexpr auto ACTIVE_BRAKE_CURRENT_LIMIT = Field<Register::REV_DRIVE_CONFIG, 20, 3, uint8_t>();
constexpr auto ACTIVE_BRAKE_KP = Field<Register::REV_DRIVE_CONFIG, 10, 10, uint16_t>();
constexpr auto ACTIVE_BRAKE_KI = Field<Register::REV_DRIVE_CONFIG, 0, 10, uint16_t>();
#define MCF8316_FOR_EACH_REV_DRIVE_CONFIG_FIELD(x) \
    x(REV_DRV_OPEN_LOOP_ACCEL_A1) x(REV_DRV_OPEN_LOOP_ACCEL_A2) \
    x(ACTIVE_BRAKE_CURRENT_LIMIT) x(ACTIVE_BRAKE_KP) x(ACTIVE_BRAKE_KI)

// MOTOR_STARTUP1
enum class CurrentLimit : uint8_t {
  LIMIT_0_125_A = 0, // 0.125 A
  LIMIT_0_25_A = 1,  // 0.25 A
  LIMIT_0_5_A = 2,   // 0.5 A
  LIMIT_1_0_A = 3,   // 1.0 A
  LIMIT_1_5_A = 4,   // 1.5 A
  LIMIT_2_0_A = 5,   // 2.0 A
  LIMIT_2_5_A = 6,   // 2.5 A
  LIMIT_3_0_A = 7,   // 3.0 A
  LIMIT_3_5_A = 8,   // 3.5 A
  LIMIT_4_0_A = 9,   // 4.0 A
  LIMIT_4_5_A = 10,   // 4.5 A
  LIMIT_5_0_A = 11,   // 5.0 A
  LIMIT_5_5_A = 12,   // 5.5 A
  LIMIT_6_0_A = 13,   // 6.0 A
  LIMIT_7_0_A = 14,   // 7.0 A
  LIMIT_8_0_A = 15,   // 8.0 A
};
enum class IPDClockFrequency : uint8_t {
  FREQ_50_HZ = 0,     // 50 Hz
  FREQ_100_HZ = 1,    // 100 Hz
  FREQ_250_HZ = 2,    // 250 Hz
  FREQ_500_HZ = 3,    // 500 Hz
  FREQ_1000_HZ = 4,   // 1000 Hz
  FREQ_2000_HZ = 5,   // 2000 Hz
  FREQ_5000_HZ = 6,   // 5000 Hz
  FREQ_10000_HZ = 7,  // 10000 Hz
};
enum class IPDCurrentThreshold : uint8_t {
  THR_0_25_A = 0,   // 0.25 A
  THR_0_5_A = 1,    // 0.5 A
  THR_0_75_A = 2,   // 0.75 A
  THR_1_0_A = 3,    // 1.0 A
  THR_1_25_A = 4,   // 1.25 A
  THR_1_5_A = 5,    // 1.5 A
  THR_2_0_A = 6,    // 2.0 A
  THR_2_5_A = 7,    // 2.5 A
  THR_3_0_A = 8,    // 3.0 A
  THR_3_667_A = 9,  // 3.667 A
  THR_4_0_A = 10,   // 4.0 A
  THR_4_667_A = 11, // 4.667 A
  THR_5_0_A = 12,   // 5.0 A
  THR_5_333_A = 13, // 5.333 A
  THR_6_0_A = 14,   // 6.0 A
  THR_6_667_A = 15, // 6.667 A
  THR_7_333_A = 16, // 7.333 A
  THR_8_0_A = 17,   // 8.0 A
};
constexpr auto MTR_STARTUP = Field<Register::MOTOR_STARTUP1, 29, 2, uint8_t>();
constexpr auto ALIGN_SLOW_RAMP_RATE = Field<Register::MOTOR_STARTUP1, 25, 4, uint8_t>();
constexpr auto ALIGN_TIME = Field<Register::MOTOR_STARTUP1, 21, 4, uint8_t>();
constexpr auto ALIGN_OR_SLOW_CURRENT_ILIMIT = Field<Register::MOTOR_STARTUP1, 17, 4, CurrentLimit>();
constexpr auto IPD_CLK_FREQ = Field<Register::MOTOR_STARTUP1, 14, 3, IPDClockFrequency>();
constexpr auto IPD_CURR_THR = Field<Register::MOTOR_STARTUP1, 9, 5, IPDCurrentThreshold>();
constexpr auto IPD_RLS_MODE = Field<Register::MOTOR_STARTUP1, 8, 1, uint8_t>();
constexpr auto IPD_ADV_ANGLE = Field<Register::MOTOR_STARTUP1, 6, 2, uint8_t>();
constexpr auto IPD_REPEAT = Field<Register::MOTOR_STARTUP1, 4, 2, uint8_t>();
constexpr auto IQ_RAMP_EN = Field<Register::MOTOR_STARTUP1, 2, 1, bool>();
constexpr auto ACTIVE_BRAKE_EN = Field<Register::MOTOR_STARTUP1, 1, 1, bool>();
constexpr auto REV_DRV_CONFIG = Field<Register::MOTOR_STARTUP1, 0, 1, uint8_t>();
#define MCF8316_FOR_EACH_MOTOR_STARTUP1_FIELD(x) \
    x(MTR_STARTUP) x(ALIGN_SLOW_RAMP_RATE) x(ALIGN_TIME) x(ALIGN_OR_SLOW_CURRENT_ILIMIT) \
    x(IPD_CLK_FREQ) x(IPD_CURR_THR) x(IPD_RLS_MODE) x(IPD_ADV_ANGLE) x(IPD_REPEAT) \
    x(IQ_RAMP_EN) x(ACTIVE_BRAKE_EN) x(REV_DRV_CONFIG)

// MOTOR_STARTUP2
enum class OpenLoopAcceleration {
  ACCEL_0_1_HZ_S = 0,       // 0.1 Hz/s
  ACCEL_0_5_HZ_S = 1,       // 0.5 Hz/s
  ACCEL_1_0_HZ_S = 2,       // 1.0 Hz/s
  ACCEL_2_5_HZ_S = 3,       // 2.5 Hz/s
  ACCEL_5_0_HZ_S = 4,       // 5.0 Hz/s
  ACCEL_10_0_HZ_S = 5,      // 10.0 Hz/s
  ACCEL_25_0_HZ_S = 6,      // 25.0 Hz/s
  ACCEL_50_0_HZ_S = 7,      // 50.0 Hz/s
  ACCEL_75_0_HZ_S = 8,      // 75.0 Hz/s
  ACCEL_100_0_HZ_S = 9,     // 100.0 Hz/s
  ACCEL_250_0_HZ_S = 10,    // 250.0 Hz/s
  ACCEL_500_0_HZ_S = 11,    // 500.0 Hz/s
  ACCEL_750_0_HZ_S = 12,    // 750.0 Hz/s
  ACCEL_1000_0_HZ_S = 13,   // 1000.0 Hz/s
  ACCEL_5000_0_HZ_S = 14,   // 5000.0 Hz/s
  ACCEL_10000_0_HZ_S = 15,  // 10000.0 Hz/s
};
constexpr auto OL_ILIMIT = Field<Register::MOTOR_STARTUP2, 27, 4, CurrentLimit>();
constexpr auto OL_ACC_A1 = Field<Register::MOTOR_STARTUP2, 23, 4, OpenLoopAcceleration>();
constexpr auto OL_ACC_A2 = Field<Register::MOTOR_STARTUP2, 19, 4, uint8_t>();
constexpr auto AUTO_HANDOFF_EN = Field<Register::MOTOR_STARTUP2, 18, 1, bool>();
constexpr auto OPN_CL_HANDOFF_THR = Field<Register::MOTOR_STARTUP2, 13, 5, uint8_t>();
constexpr auto ALIGN_ANGLE = Field<Register::MOTOR_STARTUP2, 8, 5, uint8_t>();
constexpr auto SLOW_FIRST_CYC_FREQ = Field<Register::MOTOR_STARTUP2, 4, 4, uint8_t>();
constexpr auto FIRST_CYCLE_FREQ_SEL = Field<Register::MOTOR_STARTUP2, 3, 1, uint8_t>();
constexpr auto THETA_ERROR_RAMP_RATE = Field<Register::MOTOR_STARTUP2, 0, 3, uint8_t>();
#define MCF8316_FOR_EACH_MOTOR_STARTUP2_FIELD(x) \
    x(OL_ILIMIT) x(OL_ACC_A1) x(OL_ACC_A2) x(AUTO_HANDOFF_EN) x(OPN_CL_HANDOFF_THR) \
    x(ALIGN_ANGLE) x(SLOW_FIRST_CYC_FREQ) x(FIRST_CYCLE_FREQ_SEL) x(THETA_ERROR_RAMP_RATE)

// CLOSED_LOOP1
enum class ClosedLoopAcceleration {
  ACCEL_0_5_HZ_S = 0,       // 0.5 Hz/s
  ACCEL_1_0_HZ_S = 1,       // 1.0 Hz/s
  ACCEL_2_5_HZ_S = 2,       // 2.5 Hz/s
  ACCEL_5_0_HZ_S = 3,       // 5.0 Hz/s
  ACCEL_7_5_HZ_S = 4,       // 7.5 Hz/s
  ACCEL_10_0_HZ_S = 5,      // 10.0 Hz/s
  ACCEL_20_0_HZ_S = 6,      // 20.0 Hz/s
  ACCEL_40_0_HZ_S = 7,      // 40.0 Hz/s
  ACCEL_60_0_HZ_S = 8,      // 60.0 Hz/s
  ACCEL_80_0_HZ_S = 9,      // 80.0 Hz/s
  ACCEL_100_0_HZ_S = 10,    // 100.0 Hz/s
  ACCEL_200_0_HZ_S = 11,    // 200.0 Hz/s
  ACCEL_300_0_HZ_S = 12,    // 300.0 Hz/s
  ACCEL_400_0_HZ_S = 13,    // 400.0 Hz/s
  ACCEL_500_0_HZ_S = 14,    // 500.0 Hz/s
  ACCEL_600_0_HZ_S = 15,    // 600.0 Hz/s
  ACCEL_700_0_HZ_S = 16,    // 700.0 Hz/s
  ACCEL_800_0_HZ_S = 17,    // 800.0 Hz/s
  ACCEL_900_0_HZ_S = 18,    // 9000.0 Hz/s
  ACCEL_1000_0_HZ_S = 19,   // 1000.0 Hz/s
  ACCEL_2000_0_HZ_S = 20,   // 2000.0 Hz/s
  ACCEL_4000_0_HZ_S = 21,   // 4000.0 Hz/s
  ACCEL_6000_0_HZ_S = 22,   // 6000.0 Hz/s
  ACCEL_8000_0_HZ_S = 23,   // 8000.0 Hz/s
  ACCEL_10000_0_HZ_S = 24,  // 10000.0 Hz/s
  ACCEL_20000_0_HZ_S = 25,  // 20000.0 Hz/s
  ACCEL_30000_0_HZ_S = 26,  // 30000.0 Hz/s
  ACCEL_40000_0_HZ_S = 27,  // 40000.0 Hz/s
  ACCEL_50000_0_HZ_S = 28,  // 50000.0 Hz/s
  ACCEL_60000_0_HZ_S = 29,  // 60000.0 Hz/s
  ACCEL_70000_0_HZ_S = 30,  // 70000.0 Hz/s
  ACCEL_NO_LIMIT = 31,      // No limit
};
using ClosedLoopDeceleration = ClosedLoopAcceleration;
constexpr auto OVERMODULATION_ENABLE = Field<Register::CLOSED_LOOP1, 30, 1, bool>();
constexpr auto CL_ACC = Field<Register::CLOSED_LOOP1, 25, 5, ClosedLoopAcceleration>();
constexpr auto CL_DEC = Field<Register::CLOSED_LOOP1, 19, 5, ClosedLoopDeceleration>();
constexpr auto PWM_FREQ_OUT = Field<Register::CLOSED_LOOP1, 15, 4, uint8_t>();
constexpr auto PWM_MODE = Field<Register::CLOSED_LOOP1, 14, 1, uint8_t>();
constexpr auto FG_SEL = Field<Register::CLOSED_LOOP1, 12, 2, uint8_t>();
constexpr auto FG_DIV = Field<Register::CLOSED_LOOP1, 8, 4, uint8_t>();
constexpr auto FG_CONFIG = Field<Register::CLOSED_LOOP1, 7, 1, uint8_t>();
constexpr auto FG_BEMF_THR = Field<Register::CLOSED_LOOP1, 4, 3, uint8_t>();
constexpr auto AVS_EN = Field<Register::CLOSED_LOOP1, 3, 1, bool>();
constexpr auto DEADTIME_COMP_EN = Field<Register::CLOSED_LOOP1, 2, 1, bool>();
constexpr auto LOW_SPEED_RECIRC_BRAKE_EN = Field<Register::CLOSED_LOOP1, 0, 1, bool>();
#define MCF8316_FOR_EACH_CLOSED_LOOP1_FIELD(x) \
    x(OVERMODULATION_ENABLE) x(CL_ACC) x(CL_DEC) x(PWM_FREQ_OUT) x(PWM_MODE) \
    x(FG_SEL) x(FG_DIV) x(FG_CONFIG) x(FG_BEMF_THR) x(AVS_EN) x(DEADTIME_COMP_EN) \
    x(LOW_SPEED_RECIRC_BRAKE_EN)

// CLOSED_LOOP2
constexpr auto MTR_STOP = Field<Register::CLOSED_LOOP2, 28, 3, uint8_t>();
constexpr auto MTR_STOP_BRK_TIME = Field<Register::CLOSED_LOOP2, 24, 4, uint8_t>();
constexpr auto ACT_SPIN_THR = Field<Register::CLOSED_LOOP2, 20, 4, uint8_t>();
constexpr auto BRAKE_SPEED_THRESHOLD = Field<Register::CLOSED_LOOP2, 16, 4, uint8_t>();
constexpr auto MOTOR_RES = Field<Register::CLOSED_LOOP2, 8, 8, uint8_t>();
constexpr auto MOTOR_IND = Field<Register::CLOSED_LOOP2, 0, 8, uint8_t>();
#define MCF8316_FOR_EACH_CLOSED_LOOP2_FIELD(x) \
    x(MTR_STOP) x(MTR_STOP_BRK_TIME) x(ACT_SPIN_THR) x(BRAKE_SPEED_THRESHOLD) \
    x(MOTOR_RES) x(MOTOR_IND)

// CLOSED_LOOP3
constexpr auto MOTOR_BEMF_CONST = Field<Register::CLOSED_LOOP3, 23, 8, uint8_t>();
constexpr auto CURR_LOOP_KP = Field<Register::CLOSED_LOOP3, 13, 10, uint16_t>();
constexpr auto CURR_LOOP_KI = Field<Register::CLOSED_LOOP3, 3, 10, uint16_t>();
constexpr auto SPD_LOOP_KP_MSB = Field<Register::CLOSED_LOOP3, 0, 3, uint16_t>();
#define MCF8316_FOR_EACH_CLOSED_LOOP3_FIELD(x) \
    x(MOTOR_BEMF_CONST) x(CURR_LOOP_KP) x(CURR_LOOP_KI) x(SPD_LOOP_KP_MSB)

// CLOSED_LOOP4
constexpr auto SPD_LOOP_KP_LSB = Field<Register::CLOSED_LOOP4, 24, 7, uint16_t>();
constexpr auto SPD_LOOP_KI = Field<Register::CLOSED_LOOP4, 14, 10, uint16_t>();
constexpr auto MAX_SPEED = Field<Register::CLOSED_LOOP4, 0, 14, uint16_t>();
#define MCF8316_FOR_EACH_CLOSED_LOOP4_FIELD(x) \
    x(SPD_LOOP_KP_LSB) x(SPD_LOOP_KI) x(MAX_SPEED)

// REF_PROFILES1
constexpr auto REF_PROFILE_CONFIG = Field<Register::REF_PROFILES1, 29, 2, uint8_t>();
constexpr auto DUTY_ON1 = Field<Register::REF_PROFILES1, 21, 8, uint8_t>();
constexpr auto DUTY_OFF1 = Field<Register::REF_PROFILES1, 13, 8, uint8_t>();
constexpr auto DUTY_CLAMP1 = Field<Register::REF_PROFILES1, 5, 8, uint8_t>();
constexpr auto DUTY_A_MSB = Field<Register::REF_PROFILES1, 0, 5, uint8_t>();
#define MCF8316_FOR_EACH_REF_PROFILES1_FIELD(x) \
    x(REF_PROFILE_CONFIG) x(DUTY_ON1) x(DUTY_OFF1) x(DUTY_CLAMP1) x(DUTY_A_MSB)

// REF_PROFILES2
constexpr auto DUTY_A_LSB = Field<Register::REF_PROFILES2, 28, 3, uint8_t>();
constexpr auto DUTY_B = Field<Register::REF_PROFILES2, 20, 8, uint8_t>();
constexpr auto DUTY_C = Field<Register::REF_PROFILES2, 12, 8, uint8_t>();
constexpr auto DUTY_D = Field<Register::REF_PROFILES2, 4, 8, uint8_t>();
constexpr auto DUTY_E_MSB = Field<Register::REF_PROFILES2, 0, 4, uint8_t>();
#define MCF8316_FOR_EACH_REF_PROFILES2_FIELD(x) \
    x(DUTY_A_LSB) x(DUTY_B) x(DUTY_C) x(DUTY_D) x(DUTY_E_MSB)

// REF_PROFILES3
constexpr auto DUTY_E_LSB = Field<Register::REF_PROFILES3, 27, 4, uint8_t>();
constexpr auto DUTY_ON2 = Field<Register::REF_PROFILES3, 19, 8, uint8_t>();
constexpr auto DUTY_OFF2 = Field<Register::REF_PROFILES3, 11, 8, uint8_t>();
constexpr auto DUTY_CLAMP2 = Field<Register::REF_PROFILES3, 3, 8, uint8_t>();
constexpr auto DUTY_HYS = Field<Register::REF_PROFILES3, 1, 2, uint8_t>();
#define MCF8316_FOR_EACH_REF_PROFILES3_FIELD(x) \
    x(DUTY_E_LSB) x(DUTY_ON2) x(DUTY_OFF2) x(DUTY_CLAMP2) x(DUTY_HYS)

// REF_PROFILES4
constexpr auto REF_OFF1 = Field<Register::REF_PROFILES4, 23, 8, uint8_t>();
constexpr auto REF_CLAMP1 = Field<Register::REF_PROFILES4, 15, 8, uint8_t>();
constexpr auto REF_A = Field<Register::REF_PROFILES4, 7, 8, uint8_t>();
constexpr auto REF_B_MSB = Field<Register::REF_PROFILES4, 0, 7, uint8_t>();
#define MCF8316_FOR_EACH_REF_PROFILES4_FIELD(x) \
    x(REF_OFF1) x(REF_CLAMP1) x(REF_A) x(REF_B_MSB)

// REF_PROFILES5
constexpr auto REF_B_LSB = Field<Register::REF_PROFILES5, 30, 1, uint8_t>();
constexpr auto REF_C = Field<Register::REF_PROFILES5, 22, 8, uint8_t>();
constexpr auto REF_D = Field<Register::REF_PROFILES5, 14, 8, uint8_t>();
constexpr auto REF_E = Field<Register::REF_PROFILES5, 6, 8, uint8_t>();
constexpr auto MIN_DUTY = Field<Register::REF_PROFILES5, 4, 2, uint8_t>();
constexpr auto VOLTAGE_MODE_CONFIG = Field<Register::REF_PROFILES5, 2, 2, uint8_t>();
constexpr auto DUTY_COMMAND_FILTER = Field<Register::REF_PROFILES5, 1, 1, uint8_t>();
#define MCF8316_FOR_EACH_REF_PROFILES5_FIELD(x) \
    x(REF_B_LSB) x(REF_C) x(REF_D) x(REF_E) x(MIN_DUTY) x(VOLTAGE_MODE_CONFIG) \
    x(DUTY_COMMAND_FILTER)

// REF_PROFILES6
constexpr auto REF_OFF2 = Field<Register::REF_PROFILES6, 23, 8, uint8_t>();
constexpr auto REF_CLAMP2 = Field<Register::REF_PROFILES6, 15, 8, uint8_t>();
#define MCF8316_FOR_EACH_REF_PROFILES6_FIELD(x) \
    x(REF_OFF2) x(REF_CLAMP2)

// FAULT_CONFIG1
constexpr auto ILIMIT = Field<Register::FAULT_CONFIG1, 27, 4, CurrentLimit>();
constexpr auto HW_LOCK_ILIMIT = Field<Register::FAULT_CONFIG1, 23, 4, CurrentLimit>();
constexpr auto LOCK_ILIMIT = Field<Register::FAULT_CONFIG1, 19, 4, CurrentLimit>();
constexpr auto EEP_FAULT_MODE = Field<Register::FAULT_CONFIG1, 18, 1, uint8_t>();
constexpr auto LOCK_ILIMIT_MODE = Field<Register::FAULT_CONFIG1, 15, 3, uint8_t>();
constexpr auto LOCK_ILIMIT_DEG = Field<Register::FAULT_CONFIG1, 11, 4, uint8_t>();
constexpr auto LCK_RETRY = Field<Register::FAULT_CONFIG1, 7, 4, uint8_t>();
constexpr auto CRC_ERR_MODE = Field<Register::FAULT_CONFIG1, 6, 1, uint8_t>();
constexpr auto MTR_LCK_MODE = Field<Register::FAULT_CONFIG1, 3, 3, uint8_t>();
constexpr auto IPD_TIMEOUT_FAULT_EN = Field<Register::FAULT_CONFIG1, 2, 1, bool>();
constexpr auto IPD_FREQ_FAULT_EN = Field<Register::FAULT_CONFIG1, 1, 1, bool>();
constexpr auto SATURATION_FLAGS_EN = Field<Register::FAULT_CONFIG1, 0, 1, bool>();
#define MCF8316_FOR_EACH_FAULT_CONFIG1_FIELD(x) \
    x(ILIMIT) x(HW_LOCK_ILIMIT) x(LOCK_ILIMIT) x(EEP_FAULT_MODE) x(LOCK_ILIMIT_MODE) \
    x(LOCK_ILIMIT_DEG) x(LCK_RETRY) x(CRC_ERR_MODE) x(MTR_LCK_MODE) x(IPD_TIMEOUT_FAULT_EN) \
    x(IPD_FREQ_FAULT_EN) x(SATURATION_FLAGS_EN)

// FAULT_CONFIG2
constexpr auto LOCK1_EN = Field<Register::FAULT_CONFIG2, 30, 1, bool>();
constexpr auto LOCK2_EN = Field<Register::FAULT_CONFIG2, 29, 1, bool>();
constexpr auto LOCK3_EN = Field<Register::FAULT_CONFIG2, 28, 1, bool>();
constexpr auto LOCK_ABN_SPEED = Field<Register::FAULT_CONFIG2, 25, 3, uint8_t>();
constexpr auto ABNORMAL_BEMF_THR = Field<Register::FAULT_CONFIG2, 22, 3, uint8_t>();
constexpr auto NO_MTR_THR = Field<Register::FAULT_CONFIG2, 19, 3, uint8_t>();
constexpr auto HW_LOCK_ILIMIT_MODE = Field<Register::FAULT_CONFIG2, 16, 3, uint8_t>();
constexpr auto HW_LOCK_ILIMIT_DEG = Field<Register::FAULT_CONFIG2, 13, 3, uint8_t>();
constexpr auto VOLTAGE_HYSTERESIS = Field<Register::FAULT_CONFIG2, 11, 2, uint8_t>();
constexpr auto MIN_VM_MOTOR = Field<Register::FAULT_CONFIG2, 8, 3, uint8_t>();
constexpr auto MIN_VM_MODE = Field<Register::FAULT_CONFIG2, 7, 1, uint8_t>();
constexpr auto MAX_VM_MOTOR = Field<Register::FAULT_CONFIG2, 4, 3, uint8_t>();
constexpr auto MAX_VM_MODE = Field<Register::FAULT_CONFIG2, 3, 1, uint8_t>();
constexpr auto AUTO_RETRY_TIMES = Field<Register::FAULT_CONFIG2, 0, 3, uint8_t>();
#define MCF8316_FOR_EACH_FAULT_CONFIG2_FIELD(x) \
    x(LOCK1_EN) x(LOCK2_EN) x(LOCK3_EN) x(LOCK_ABN_SPEED) x(ABNORMAL_BEMF_THR) \
    x(NO_MTR_THR) x(HW_LOCK_ILIMIT_MODE) x(HW_LOCK_ILIMIT_DEG) x(VOLTAGE_HYSTERESIS) \
    x(MIN_VM_MOTOR) x(MIN_VM_MODE) x(MAX_VM_MOTOR) x(MAX_VM_MODE) x(AUTO_RETRY_TIMES)

// PIN_CONFIG
enum class BrakeInput : uint8_t {
  BRAKE_PIN = 0,
  OVERRIDE_ON = 1,
  OVERRIDE_OFF = 2,
  BRAKE_PIN2 = 3, // same behavior as BRAKE_PIN
};
enum class SpeedMode : uint8_t {
  SPEED_PIN_ANALOG = 0,
  SPEED_PIN_DUTY_CYCLE = 1,
  DIGITAL_SPEED_CTRL = 2,
  SPEED_PIN_FREQUENCY = 3,
};
constexpr auto PWM_DITHER_STEP = Field<Register::PIN_CONFIG, 29, 2, uint8_t>();
constexpr auto VDC_FILTER = Field<Register::PIN_CONFIG, 27, 2, uint8_t>();
constexpr auto LEAD_ANGLE = Field<Register::PIN_CONFIG, 22, 5, uint8_t>();
constexpr auto MAX_POWER = Field<Register::PIN_CONFIG, 11, 14, uint16_t>();
constexpr auto FG_IDLE_CONFIG = Field<Register::PIN_CONFIG, 9, 2, uint8_t>();
constexpr auto FG_FAULT_CONFIG = Field<Register::PIN_CONFIG, 7, 2, uint8_t>();
constexpr auto ALARM_PIN_EN = Field<Register::PIN_CONFIG, 6, 1, bool>();
constexpr auto BRAKE_PIN_MODE = Field<Register::PIN_CONFIG, 5, 1, uint8_t>();
constexpr auto ALIGN_BRAKE_ANGLE_SEL = Field<Register::PIN_CONFIG, 4, 1, uint8_t>();
constexpr auto BRAKE_INPUT = Field<Register::PIN_CONFIG, 2, 2, BrakeInput>();
constexpr auto SPEED_MODE = Field<Register::PIN_CONFIG, 0, 2, SpeedMode>();
#define MCF8316_FOR_EACH_PIN_CONFIG_FIELD(x) \
    x(PWM_DITHER_DEPTH) x(VDC_FILTER) x(LEAD_ANGLE) x(MAX_POWER) x(FG_IDLE_CONFIG) \
    x(FG_FAULT_CONFIG) x(ALARM_PIN_EN) x(BRAKE_PIN_MODE) x(ALIGN_BRAKE_ANGLE_SEL) \
    x(BRAKE_INPUT) x(SPEED_MODE)

// DEVICE_CONFIG1
constexpr auto DAC_SOx_SEL = Field<Register::DEVICE_CONFIG1, 28, 2, uint8_t>();
constexpr auto PWM_DITHER_MODE = Field<Register::DEVICE_CONFIG1, 27, 1, uint8_t>();
constexpr auto I2C_TARGET_ADDR = Field<Register::DEVICE_CONFIG1, 20, 7, uint8_t>();
constexpr auto EEPROM_LOCK_KEY = Field<Register::DEVICE_CONFIG1, 5, 15, uint16_t>();
constexpr auto SLEW_RATE_I2C_PINS = Field<Register::DEVICE_CONFIG1, 3, 2, uint8_t>();
constexpr auto PULLUP_ENABLE = Field<Register::DEVICE_CONFIG1, 2, 1, bool>();
constexpr auto BUS_VOLT = Field<Register::DEVICE_CONFIG1, 0, 2, uint8_t>();
#define MCF8316_FOR_EACH_DEVICE_CONFIG1_FIELD(x) \
    x(DAC_SOx_SEL) x(PWM_DITHER_MODE) x(I2C_TARGET_ADDR) x(EEPROM_LOCK_KEY) \
    x(SLEW_RATE_I2C_PINS) x(PULLUP_ENABLE) x(BUS_VOLT)

// DEVICE_CONFIG2
constexpr auto INPUT_MAXIMUM_FREQ = Field<Register::DEVICE_CONFIG2, 16, 15, uint16_t>();
constexpr auto SLEEP_ENTRY_TIME = Field<Register::DEVICE_CONFIG2, 14, 2, uint8_t>();
constexpr auto DYNAMIC_CSA_GAIN_EN = Field<Register::DEVICE_CONFIG2, 13, 1, bool>();
constexpr auto DYNAMIC_VOLTAGE_GAIN_EN = Field<Register::DEVICE_CONFIG2, 12, 1, bool>();
constexpr auto DEV_MODE = Field<Register::DEVICE_CONFIG2, 11, 1, uint8_t>();
constexpr auto PWM_DITHER_DEPTH = Field<Register::DEVICE_CONFIG2, 9, 2, uint8_t>();
constexpr auto EXT_CLK_EN = Field<Register::DEVICE_CONFIG2, 8, 1, bool>();
constexpr auto EXT_CLK_CONFIG = Field<Register::DEVICE_CONFIG2, 5, 3, uint8_t>();
constexpr auto EXT_WDT_EN = Field<Register::DEVICE_CONFIG2, 4, 1, bool>();
constexpr auto EXT_WDT_CONFIG = Field<Register::DEVICE_CONFIG2, 2, 2, uint8_t>();
constexpr auto EXT_WDT_INPUT_MODE = Field<Register::DEVICE_CONFIG2, 1, 1, uint8_t>();
constexpr auto EXT_WDT_FAULT_MODE = Field<Register::DEVICE_CONFIG2, 0, 1, uint8_t>();
#define MCF8316_FOR_EACH_DEVICE_CONFIG2_FIELD(x) \
    x(INPUT_MAXIMUM_FREQ) x(SLEEP_ENTRY_TIME) x(DYNAMIC_CSA_GAIN_EN) x(DYNAMIC_VOLTAGE_GAIN_EN) \
    x(DEV_MODE) x(PWM_DITHER_DEPTH) x(EXT_CLK_EN) x(EXT_CLK_CONFIG) x(EXT_WDT_EN) \
    x(EXT_WDT_CONFIG) x(EXT_WDT_INPUT_MODE) x(EXT_WDT_FAULT_MODE)

// PERI_CONFIG1
enum class DirInput : uint8_t {
  DIR_PIN = 0,
  OVERRIDE_CLOCKWISE = 1,
  OVERRIDE_COUNTER_CLOCKWISE = 2,
  DIR_PIN2 = 3, // same behavior as DIR_PIN
};
constexpr auto SPREAD_SPECTRUM_MODULATION_DIS = Field<Register::PERI_CONFIG1, 30, 1, bool>();
constexpr auto NO_MTR_FLT_CLOSEDLOOP_DIS = Field<Register::PERI_CONFIG1, 28, 1, bool>();
constexpr auto ABNORMAL_BEMF_PERSISTENT_TIME = Field<Register::PERI_CONFIG1, 26, 2, uint8_t>();
constexpr auto FLUX_WEAK_REF = Field<Register::PERI_CONFIG1, 24, 2, uint8_t>();
constexpr auto INPUT_REFERENCE_WINDOW = Field<Register::PERI_CONFIG1, 22, 2, uint8_t>();
constexpr auto BUS_POWER_LIMIT_ENABLE = Field<Register::PERI_CONFIG1, 21, 1, bool>();
constexpr auto DIR_INPUT = Field<Register::PERI_CONFIG1, 19, 2, DirInput>();
constexpr auto DIR_CHANGE_MODE = Field<Register::PERI_CONFIG1, 18, 1, uint8_t>();
constexpr auto SPEED_LIMIT_ENABLE = Field<Register::PERI_CONFIG1, 17, 1, bool>();
constexpr auto ACTIVE_BRAKE_SPEED_DELTA_LIMIT_ENTRY = Field<Register::PERI_CONFIG1, 13, 4, uint8_t>();
constexpr auto ACTIVE_BRAKE_MOD_INDEX_LIMIT = Field<Register::PERI_CONFIG1, 10, 3, uint8_t>();
constexpr auto SPEED_RANGE_SEL = Field<Register::PERI_CONFIG1, 9, 1, uint8_t>();
constexpr auto INPUT_REFERENCE_MODE = Field<Register::PERI_CONFIG1, 7, 2, uint8_t>();
constexpr auto EEPROM_LOCK_MODE = Field<Register::PERI_CONFIG1, 5, 2, uint8_t>();
#define MCF8316_FOR_EACH_PERI_CONFIG1_FIELD(x) \
    x(SPREAD_SPECTRUM_MODULATION_DIS) x(NO_MTR_FLT_CLOSEDLOOP_DIS) x(ABNORMAL_BEMF_PERSISTENT_TIME) \
    x(FLUX_WEAK_REF) x(INPUT_REFERENCE_WINDOW) x(BUS_POWER_LIMIT_ENABLE) x(DIR_INPUT) \
    x(DIR_CHANGE_MODE) x(SPEED_LIMIT_ENABLE) x(ACTIVE_BRAKE_SPEED_DELTA_LIMIT_ENTRY) \
    x(ACTIVE_BRAKE_MOD_INDEX_LIMIT) x(SPEED_RANGE_SEL) x(INPUT_REFERENCE_MODE) \
    x(EEPROM_LOCK_MODE)

// GD_CONFIG1
constexpr auto SLEW_RATE = Field<Register::GD_CONFIG1, 26, 2, uint8_t>();
constexpr auto OVP_SEL = Field<Register::GD_CONFIG1, 19, 1, uint8_t>();
constexpr auto OVP_EN = Field<Register::GD_CONFIG1, 18, 1, bool>();
constexpr auto OTW_REP = Field<Register::GD_CONFIG1, 16, 1, bool>();
constexpr auto OCP_DEG = Field<Register::GD_CONFIG1, 12, 2, uint8_t>();
constexpr auto OCP_LVL = Field<Register::GD_CONFIG1, 10, 1, uint8_t>();
constexpr auto OCP_MODE = Field<Register::GD_CONFIG1, 8, 2, uint8_t>();
constexpr auto CSA_GAIN = Field<Register::GD_CONFIG1, 0, 2, uint8_t>();
#define MCF8316_FOR_EACH_GD_CONFIG1_FIELD(x) \
    x(SLEW_RATE) x(OVP_SEL) x(OVP_EN) x(OTW_REP) x(OCP_DEG) x(OCP_LVL) \
    x(OCP_MODE) x(CSA_GAIN)

// GD_CONFIG2
constexpr auto BUCK_PS_DIS = Field<Register::GD_CONFIG2, 24, 1, bool>();
constexpr auto BUCK_CL = Field<Register::GD_CONFIG2, 23, 1, uint8_t>();
constexpr auto BUCK_SEL = Field<Register::GD_CONFIG2, 21, 2, uint8_t>();
constexpr auto BUCK_DIS = Field<Register::GD_CONFIG2, 20, 1, bool>();
constexpr auto MIN_ON_TIME = Field<Register::GD_CONFIG2, 17, 3, uint8_t>();
#define MCF8316_FOR_EACH_GD_CONFIG2_FIELD(x) \
    x(BUCK_PS_DIS) x(BUCK_CL) x(BUCK_SEL) x(BUCK_DIS) x(MIN_ON_TIME)

// INT_ALGO1
constexpr auto ACTIVE_BRAKE_SPEED_DELTA_LIMIT_EXIT = Field<Register::INT_ALGO1, 29, 2, uint8_t>();
constexpr auto SPEED_PIN_GLITCH_FILTER = Field<Register::INT_ALGO1, 27, 2, uint8_t>();
constexpr auto FAST_ISD_EN = Field<Register::INT_ALGO1, 26, 1, bool>();
constexpr auto ISD_STOP_TIME = Field<Register::INT_ALGO1, 24, 2, uint8_t>();
constexpr auto ISD_RUN_TIME = Field<Register::INT_ALGO1, 22, 2, uint8_t>();
constexpr auto ISD_TIMEOUT = Field<Register::INT_ALGO1, 20, 2, uint8_t>();
constexpr auto AUTO_HANDOFF_MIN_BEMF = Field<Register::INT_ALGO1, 17, 3, uint8_t>();
constexpr auto BRAKE_CURRENT_PERSIST = Field<Register::INT_ALGO1, 15, 2, uint8_t>();
constexpr auto REV_DRV_OPEN_LOOP_DEC = Field<Register::INT_ALGO1, 0, 3, uint8_t>();
#define MCF8316_FOR_EACH_INT_ALGO1_FIELD(x) \
    x(ACTIVE_BRAKE_SPEED_DELTA_LIMIT_EXIT) x(SPEED_PIN_GLITCH_FILTER) x(FAST_ISD_EN) \
    x(ISD_STOP_TIME) x(ISD_RUN_TIME) x(ISD_TIMEOUT) x(AUTO_HANDOFF_MIN_BEMF) \
    x(BRAKE_CURRENT_PERSIST) x(REV_DRV_OPEN_LOOP_DEC)

// INT_ALGO2
enum ClosedLoopSlowAcceleration : uint8_t {
  ACCEL_0_1_HZ_S = 0,       // 0.1 Hz/s
  ACCEL_1_0_HZ_S = 1,       // 1.0 Hz/s
  ACCEL_2_0_HZ_S = 2,       // 2.0 Hz/s
  ACCEL_3_0_HZ_S = 3,       // 3.0 Hz/s
  ACCEL_5_0_HZ_S = 4,       // 5.0 Hz/s
  ACCEL_10_0_HZ_S = 5,      // 10.0 Hz/s
  ACCEL_20_0_HZ_S = 6,      // 20.0 Hz/s
  ACCEL_30_0_HZ_S = 7,      // 30.0 Hz/s
  ACCEL_40_0_HZ_S = 8,      // 40.0 Hz/s
  ACCEL_50_0_HZ_S = 9,      // 50.0 Hz/s
  ACCEL_100_0_HZ_S = 10,    // 100.0 Hz/s
  ACCEL_200_0_HZ_S = 11,    // 200.0 Hz/s
  ACCEL_500_0_HZ_S = 12,    // 500.0 Hz/s
  ACCEL_750_0_HZ_S = 13,    // 750.0 Hz/s
  ACCEL_1000_0_HZ_S = 14,    // 1000.0 Hz/s
  ACCEL_2000_0_HZ_S = 15,    // 2000.0 Hz/s
};
constexpr auto FLUX_WEAK_KP = Field<Register::INT_ALGO2, 21, 10, uint16_t>();
constexpr auto FLUX_WEAK_KI = Field<Register::INT_ALGO2, 11, 5, uint16_t>();
constexpr auto FLUX_WEAK_ENABLE = Field<Register::INT_ALGO2, 10, 1, bool>();
constexpr auto CL_SLOW_ACC = Field<Register::INT_ALGO2, 6, 4, ClosedLoopSlowAcceleration>();
constexpr auto ACTIVE_BRAKE_BUS_CURRENT_SLEW_RATE = Field<Register::INT_ALGO2, 3, 3, uint8_t>();
constexpr auto ISD_BEMF_FILT_ENABLE = Field<Register::INT_ALGO2, 2, 1, bool>();
constexpr auto CIRCULAR_CURRENT_LIMIT_ENABLE = Field<Register::INT_ALGO2, 1, 1, bool>();
constexpr auto IPD_HIGH_RESOLUTION_EN = Field<Register::INT_ALGO2, 1, 1, bool>();
#define MCF8316_FOR_EACH_INT_ALGO2_FIELD(x) \
    x(FLUX_WEAK_KP) x(FLUX_WEAK_KI) x(FLUX_WEAK_ENABLE) x(CL_SLOW_ACC) \
    x(ACTIVE_BRAKE_BUS_CURRENT_SLEW_RATE) x(ISD_BEMF_FILT_ENABLE) \
    x(CIRCULAR_CURRENT_LIMIT_ENABLE) x(IPD_HIGH_RESOLUTION_EN)

// Composite EEPROM fields split across two registers
constexpr auto SPD_LOOP_KP = make_composite_field<uint16_t>(SPD_LOOP_KP_MSB, SPD_LOOP_KP_LSB);
constexpr auto DUTY_A = make_composite_field<uint8_t>(DUTY_A_MSB, DUTY_A_LSB);
constexpr auto DUTY_E = make_composite_field<uint8_t>(DUTY_E_MSB, DUTY_E_LSB);
constexpr auto REF_B = make_composite_field<uint8_t>(REF_B_MSB, REF_B_LSB);
#define MCF8316_FOR_EACH_COMPOSITE_EEPROM_FIELD(x) \
    x(SPD_LOOP_KP) x(DUTY_A) x(DUTY_E) x(REF_B)

// Other register fields of interest (not an exhaustive list)
// MTR_PARAMS
constexpr auto MPET_MOTOR_R = Field<Register::MTR_PARAMS, 24, 8, uint8_t>();
constexpr auto MPET_MOTOR_BEMF_CONST = Field<Register::MTR_PARAMS, 16, 8, uint8_t>();
constexpr auto MPET_MOTOR_L = Field<Register::MTR_PARAMS, 8, 8, uint8_t>();

// ALGO_STATUS_MPEG
constexpr auto MPET_R_STATUS = Field<Register::ALGO_STATUS_MPET, 31, 1, bool>();
constexpr auto MPET_L_STATUS = Field<Register::ALGO_STATUS_MPET, 30, 1, bool>();
constexpr auto MPET_KE_STATUS = Field<Register::ALGO_STATUS_MPET, 29, 1, bool>();
constexpr auto MPET_MECH_STATUS = Field<Register::ALGO_STATUS_MPET, 28, 1, bool>();
constexpr auto MPET_PWM_FREQ = Field<Register::ALGO_STATUS_MPET, 24, 4, uint8_t>();

// ALGO_CTRL1
constexpr auto EEPROM_WRT = Field<Register::ALGO_CTRL1, 31, 1, bool>();
constexpr auto EEPROM_READ = Field<Register::ALGO_CTRL1, 30, 1, bool>();
constexpr auto CLR_FLT = Field<Register::ALGO_CTRL1, 29, 1, bool>();
constexpr auto CLR_FLT_RETRY_COUNT = Field<Register::ALGO_CTRL1, 28, 1, bool>();
constexpr auto EEPROM_WRITE_ACCESS_KEY = Field<Register::ALGO_CTRL1, 20, 8, uint8_t>();
constexpr auto WATCHDOG_TICKLE = Field<Register::ALGO_CTRL1, 10, 1, bool>();

// ALGO_DEBUG1
constexpr auto DIGITAL_SPEED_OVERRIDE = Field<Register::ALGO_DEBUG1, 31, 1, bool>();
constexpr auto DIGITAL_SPEED_CTRL = Field<Register::ALGO_DEBUG1, 16, 15, uint16_t>();

// ALGO_DEBUG2
constexpr auto MPET_CMD = Field<Register::ALGO_DEBUG2, 5, 1, uint8_t>();
constexpr auto MPET_R = Field<Register::ALGO_DEBUG2, 4, 1, bool>();
constexpr auto MPET_L = Field<Register::ALGO_DEBUG2, 3, 1, bool>();
constexpr auto MPET_KE = Field<Register::ALGO_DEBUG2, 2, 1, bool>();
constexpr auto MPET_MECH = Field<Register::ALGO_DEBUG2, 1, 1, bool>();
constexpr auto MPET_WRITE_SHADOW = Field<Register::ALGO_DEBUG2, 0, 1, bool>();

// CURRENT_PI (fields renamed to avoid conflict with similarly named CURR_LOOP_KI/KI)
constexpr auto CURRENT_PI_LOOP_KI = Field<Register::CURRENT_PI, 16, 16, uint16_t>();
constexpr auto CURRENT_PI_LOOP_KP = Field<Register::CURRENT_PI, 0, 16, uint16_t>();

// SPEED_PI
constexpr auto SPEED_PI_LOOP_KI = Field<Register::SPEED_PI, 16, 16, uint16_t>();
constexpr auto SPEED_PI_LOOP_KP = Field<Register::SPEED_PI, 0, 16, uint16_t>();

// ALGORITHM_STATE
enum class AlgorithmState : uint16_t {
  MOTOR_IDLE = 0x0,
  MOTOR_ISD = 0x1,
  MOTOR_TRISTATE = 0x2,
  MOTOR_BRAKE_ON_START = 0x3,
  MOTOR_IPD = 0x4,
  MOTOR_SLOW_FIRST_CYCLE = 0x5,
  MOTOR_ALIGN = 0x6,
  MOTOR_OPEN_LOOP = 0x7,
  MOTOR_CLOSED_LOOP_UNALIGNED = 0x8,
  MOTOR_CLOSED_LOOP_ALIGNED = 0x9,
  MOTOR_CLOSED_LOOP_ACTIVE_BRAKING = 0xa,
  MOTOR_SOFT_STOP = 0xb,
  MOTOR_RECIRCULATE_STOP = 0xc,
  MOTOR_BRAKE_ON_STOP = 0xd,
  MOTOR_FAULT = 0xe,
  MOTOR_MPET_MOTOR_STOP_CHECK = 0xf,
  MOTOR_MPET_MOTOR_STOP_WAIT = 0x10,
  MOTOR_MPET_MOTOR_BRAKE = 0x11,
  MOTOR_MPET_ALGORITHM_PARAMETERS_INIT = 0x12,
  MOTOR_MPET_RL_MEASURE = 0x13,
  MOTOR_MPET_KE_MEASURE = 0x14,
  MOTOR_MPET_STALL_CURRENT_MEASURE = 0x15,
  MOTOR_MPET_TORQUE_MODE = 0x16,
  MOTOR_MPET_DONE = 0x17,
  MOTOR_MPET_FAULT = 0x18,
};
bool is_mpet_running(AlgorithmState state);
const char* algorithm_state_name(AlgorithmState state);
constexpr auto ALGORITHM_STATE = Field<Register::ALGORITHM_STATE, 0, 16, AlgorithmState>();

// Holds the value of a specific register and provides type-safe access to its fields.
struct RegisterValue_ {
  uint32_t value{};

  bool operator==(const RegisterValue_&) const = default;
  bool operator!=(const RegisterValue_&) const = default;
};
template <Register reg>
struct RegisterValue final : public RegisterValue_ {
  template <unsigned bit, unsigned width, typename T>
  T get(Field<reg, bit, width, T> field) const {
    return T((this->value & field.mask) >> field.bit);
  }

  template <unsigned bit, unsigned width, typename T>
  RegisterValue& set(Field<reg, bit, width, T> field, T field_value) {
    this->value = (this->value & ~field.mask) | ((uint32_t(field_value) << field.bit) & field.mask);
    return *this;
  }
};

// Holds a copy of the configuration registers and provides type-safe access to their fields.
// These registers are loaded from EEPROM on reset (all other registers are volatile).
struct Config final {
  static constexpr size_t LENGTH = 24;

  std::array<RegisterValue_, LENGTH> register_values = {};

  static constexpr Register index_to_register(size_t index) { return Register(index * 2 + size_t(Register::ISD_CONFIG)); }
  static constexpr size_t register_to_index(Register reg) { return (size_t(reg) - size_t(Register::ISD_CONFIG)) / 2; }

  template <Register reg>
  RegisterValue<reg>& at() {
    static_assert(is_config_register(reg), "Not a configuration register");
    return *static_cast<RegisterValue<reg>*>(&this->register_values[register_to_index(reg)]);
  }

  template <Register reg>
  const RegisterValue<reg>& at() const {
    static_assert(is_config_register(reg), "Not a configuration register");
    return *static_cast<const RegisterValue<reg>*>(&this->register_values[register_to_index(reg)]);
  }

  template <Register reg, unsigned bit, unsigned width, typename T>
  T get(Field<reg, bit, width, T> field) const {
    return at<reg>().get(field);
  }

  template <Register reg, unsigned bit, unsigned width, typename T>
  Config& set(Field<reg, bit, width, T> field, T field_value) {
    at<reg>().set(field, field_value);
    return *this;
  }

  template <typename MSBField, typename LSBField, typename T>
  T get(CompositeField<MSBField, LSBField, T> field) const {
    return T((get(field.msb) << LSBField::width) | get(field.lsb));
  }

  template <typename MSBField, typename LSBField, typename T>
  Config& set(CompositeField<MSBField, LSBField, T> field, T field_value) {
    set(field.msb, typename MSBField::Type(field_value >> LSBField::width));
    set(field.lsb, typename LSBField::Type(field_value));
    return *this;
  }

  // Returns true if some motor parameters have not been explicitly configured.
  // The MCF8316D will automatically run the MPET tool when a speed command is issued
  // but it's more robust to set everything up-front.
  bool needs_mpet_for_speed_loop() const {
    return get(MOTOR_RES) == 0 || get(MOTOR_IND) == 0 || get(MOTOR_BEMF_CONST) == 0 ||
        get(SPD_LOOP_KP) == 0 || get(SPD_LOOP_KI) == 0;
  }

  bool operator==(const Config&) const = default;
  bool operator!=(const Config&) const = default;
};

// Maximum power is expressed as a fraction of 100 W.
constexpr float max_power_to_watts(unsigned max_power) {
  return float(max_power) * 100 / 2048;
}
constexpr unsigned max_power_from_watts(float watts) {
  return unsigned(std::clamp(watts * 2048 / 100, 0.f, 2047.f));
}

// The number of motor poles is between 0 and 30 and always even.
// When set to 0, speed calculations use electrical Hz instead of rotor Hz.
constexpr unsigned fg_div_to_motor_poles(unsigned fg_div) {
  return fg_div << 1;
}
constexpr unsigned fg_div_from_motor_poles(unsigned motor_poles) {
  if (motor_poles >= 0 && motor_poles <= 30u && (motor_poles & 1u) == 0u) {
    return motor_poles >> 1;
  }
  return 0; // invalid
}

// The BEMF lead angle is a value between -48 and 45 degrees in steps of 3 degrees.
constexpr int lead_angle_to_degrees(unsigned lead_angle) {
  return lead_angle < 16u ? int(lead_angle * 3u) : -int((32u - lead_angle) * 3u);
}
constexpr unsigned lead_angle_from_degrees(int degrees) {
  if (degrees >= -48 && degrees <= 45 && abs(degrees) % 3 == 0) {
    return degrees >= 0 ? unsigned(degrees) / 3u : 32u - unsigned(-degrees) / 3u;
  }
  return 0; // invalid
}

// Converts speed value from rotor Hz to electrical Hz based on `FG_DIV`.
constexpr float convert_speed_in_electrical_hz_to_rotor_hz(float speed_in_electrical_hz, uint8_t fg_div) {
  return fg_div == 0 ? speed_in_electrical_hz * 3 : speed_in_electrical_hz / fg_div;
}

// Converts speed value from rotor Hz to electrical Hz based on `FG_DIV`.
constexpr float convert_speed_in_rotor_hz_to_electrical_hz(float speed_in_rotor_hz, uint8_t fg_div) {
  return fg_div == 0 ? speed_in_rotor_hz / 3 : speed_in_rotor_hz * fg_div;
}

// Write the configuration to the log.
void log_config(const Config& config);

// Format a bit-packed fault status value to a human-readable string.
std::string format_gate_driver_fault_status(uint32_t value);
std::string format_controller_fault_status(uint32_t value);

}  // namespace mcf8316
}  // namespace esphome
