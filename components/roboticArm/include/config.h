#ifndef ARM_CONFIG_H
#define ARM_CONFIG_H

#include <cstdint>
#include "motorModule.h"

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~ Robotic Arm Config ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Robotic Arm Physical Parameters

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~ FreeRTOS Config ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#define QUEUE_SIZE_USR_CMD  10

#define TASK_PRIORITY_COMMUNICATION     5
#define TASK_PRIORITY_CONTROL           2
#define TASK_PRIORITY_KINEMATICS        3
#define TASK_PRIORITY_STEP_MONITOR      1

#define TASK_STACK_DEPTH_COMMUNICATION  4096
#define TASK_STACK_DEPTH_CONTROL        4096
#define TASK_STACK_DEPTH_KINEMATICS     4096
#define TASK_STACK_DEPTH_STEP_MONITOR   4096

#define TASK_NAME_COMMUNICATION     "Communications Driver"
#define TASK_NAME_CONTROL           "Controls Driver"
#define TASK_NAME_KINEMATICS        "Kinematics Driver"
#define TASK_NAME_STEP_MONITOR      "Motor Monitor"

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~ Motor Task Config ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#define TASK_PRIORITY_MOTOR         2
#define QUEUE_SIZE_MOTOR            5   // Size of the motor's queues
#define TASK_STACK_DEPTH_MOTOR      3072
#define MOTOR_ANGLE_TOLERANCE       2.5f
#define ALL_MOTORS_BIT_MASK         (J1S_BIT_MASK | J2S_BIT_MASK | J3S_BIT_MASK | J4S_BIT_MASK | J5S_BIT_MASK | J6S_BIT_MASK)

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~ Device Parameters ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// I2C Params
#define I2C_PORT        I2C_NUM_0  // ESP32 Port 0
#define I2C_FREQ        400000     // Hz (Standard Mode)
#define I2C_SCL_PIN     GPIO_NUM_22
#define I2C_SDA_PIN     GPIO_NUM_21

// PCA9548A Parms
#define PCA9548A_SLAVE_ADDR 0x70  // Slave address

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~ Joint Parameters ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Global Joint Params
#define STEPPER_SPEED               300.0f // deg/sec
#define STEP_MONITOR_CHECK_TIME     20.0f // Hz 

// ~~ Global AS5600 Config ~~

// Power Mode
// 00 = NOM, 01 = LPM1, 10 = LPM2, 11 = LPM3
constexpr uint8_t AS5600_PM = 0b00;

// Hystersis
// 00 = OFF, 01 = 1 LSB, 10 = 2 LSBs, 11 = 3 LSBs
constexpr uint8_t AS5600_HYST = 0b11;

/* Output Stage
 00 = analog (full range from 0% to 100% between GND and VDD, 01 = analog
(reduced range from 10% to 90% between GND and VDD, 10 = digital PWM */
constexpr uint8_t AS5600_OUTS = 0b00;

// PWM Frequency
// 00 = 115 Hz; 01 = 230 Hz; 10 = 460 Hz; 11 = 920 Hz
constexpr uint8_t AS5600_PWMF = 0b00;

// Slow Filter
// 00 = 16x ; 01 = 8x; 10 = 4x; 11 = 2x
constexpr uint8_t AS5600_SF = 0b11;

/* Fast Filter Threshold (Slow-to-fast filter / Fast-to-slow filter LSB's)
000 = slow filter only, 001 = 6/1 LSBs, 010 = 7/1 LSBs, 011 = 9/1 LSBs,100 = 18/2 LSBs, 101
= 21/2 LSBs, 110 = 24/2 LSBs, 111 = 10/4 LSBs*/
constexpr uint8_t AS5600_FTH = 0b110;

// Watchdog
// 0 = OFF, 1 = ON
constexpr uint8_t AS5600_WD = 0b0;

// Config Building
constexpr uint16_t AS5600_CONF =
    (AS5600_PM    << 0)  |  // Bits 0-1
    (AS5600_HYST  << 2)  |  // Bits 2-3
    (AS5600_OUTS  << 4)  |  // Bits 4-5
    (AS5600_PWMF  << 6)  |  // Bits 6-7
    (AS5600_SF    << 8)  |  // Bits 8-9
    (AS5600_FTH   << 10) |  // Bits 10-12
    (AS5600_WD    << 13);   // Bit 13

// ~~~~~~~~~~~~~~~~~~~~


// Joint 1 Params
#define J1S_TASK_NAME   "Motor 1 Driver"
#define J1S_PORT        PCA9548A_PORT_7
#define J1S_PIN_DIR     GPIO_NUM_19
#define J1S_PIN_STEP    GPIO_NUM_23
#define J1S_GEAR_RATIO  GEAR_RATIO_1_TO_1
#define J1S_MICROSTEP   MICROSTEPPING_FULL
#define J1S_DEG_P_STEP  1.8f
#define J1S_BIT_MASK    BIT0

// Joint 2 Params
#define J2S_TASK_NAME   "Motor 2 Driver"
#define J2S_PORT        PCA9548A_PORT_6
#define J2S_PIN_DIR     GPIO_NUM_5
#define J2S_PIN_STEP    GPIO_NUM_18
#define J2S_GEAR_RATIO  GEAR_RATIO_1_TO_1
#define J2S_MICROSTEP   MICROSTEPPING_FULL
#define J2S_DEG_P_STEP  1.8f
#define J2S_BIT_MASK    BIT1

// Joint 3 Params
#define J3S_TASK_NAME   "Motor 3 Driver"
#define J3S_PORT        PCA9548A_PORT_5
#define J3S_PIN_DIR     GPIO_NUM_2
#define J3S_PIN_STEP    GPIO_NUM_4
#define J3S_GEAR_RATIO  GEAR_RATIO_1_TO_1
#define J3S_MICROSTEP   MICROSTEPPING_FULL
#define J3S_DEG_P_STEP  1.8f
#define J3S_BIT_MASK    BIT2

// Joint 4 Params
#define J4S_TASK_NAME   "Motor 4 Driver"
#define J4S_PORT        PCA9548A_PORT_4
#define J4S_PIN_DIR     GPIO_NUM_26
#define J4S_PIN_STEP    GPIO_NUM_25
#define J4S_GEAR_RATIO  GEAR_RATIO_1_TO_1
#define J4S_MICROSTEP   MICROSTEPPING_FULL
#define J4S_DEG_P_STEP  1.8f
#define J4S_BIT_MASK    BIT3

// Joint 5 Params
#define J5S_TASK_NAME   "Motor 5 Driver"
#define J5S_PORT        PCA9548A_PORT_3
#define J5S_PIN_DIR     GPIO_NUM_14
#define J5S_PIN_STEP    GPIO_NUM_27
#define J5S_GEAR_RATIO  GEAR_RATIO_1_TO_1
#define J5S_MICROSTEP   MICROSTEPPING_FULL
#define J5S_DEG_P_STEP  1.8f
#define J5S_BIT_MASK    BIT4

// Joint 6 Params
#define J6S_TASK_NAME   "Motor 6 Driver"
#define J6S_PORT        PCA9548A_PORT_2
#define J6S_PIN_DIR     GPIO_NUM_13
#define J6S_PIN_STEP    GPIO_NUM_12
#define J6S_GEAR_RATIO  GEAR_RATIO_1_TO_1
#define J6S_MICROSTEP   MICROSTEPPING_QUARTER
#define J6S_DEG_P_STEP  1.8f
#define J6S_BIT_MASK    BIT5

// End Effector Params
#define EE_PORT PCA9548A_PORT_0

// Joint Parameter Arrays
const float DEG_PER_STEP[] = {
    J1S_DEG_P_STEP / J1S_MICROSTEP / J1S_GEAR_RATIO,
    J2S_DEG_P_STEP / J2S_MICROSTEP / J2S_GEAR_RATIO,
    J3S_DEG_P_STEP / J3S_MICROSTEP / J3S_GEAR_RATIO,
    J4S_DEG_P_STEP / J4S_MICROSTEP / J4S_GEAR_RATIO,
    J5S_DEG_P_STEP / J5S_MICROSTEP / J5S_GEAR_RATIO,
    J6S_DEG_P_STEP / J6S_MICROSTEP / J6S_GEAR_RATIO,
};

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#endif