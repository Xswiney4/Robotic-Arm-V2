#ifndef ARM_CONFIG_H
#define ARM_CONFIG_H

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~ Robotic Arm Config ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Robotic Arm Physical Parameters

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~ Device Parameters ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// I2C Params
#define I2C_PORT I2C_NUM_0  // ESP32 Port 0
#define I2C_FREQ 100000     // Hz (Standard Mode)

// PCA9548A Parms
#define PCA9548A_SLAVE_ADDR 0x70  // Slave address

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~ Joint Parameters ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Global Joint Params
#define STEPPER_UPDATE_RESOLUTION 5 // updates/degree (servo smoothness)
#define STEPPER_SPEED 90 // deg/sec

// Joint 1 Params
#define J1S_PORT        7
#define J1S_PIN_DIR     GPIO_NUM_19
#define J1S_PIN_STEP    GPIO_NUM_23

// Joint 2 Params
#define J2S_PORT        6
#define J2S_PIN_DIR     GPIO_NUM_5
#define J2S_PIN_STEP    GPIO_NUM_18

// Joint 3 Params
#define J3S_PORT        5
#define J3S_PIN_DIR     GPIO_NUM_2
#define J3S_PIN_STEP    GPIO_NUM_4

// Joint 4 Params
#define J4S_PORT        4
#define J4S_PIN_DIR     GPIO_NUM_26
#define J4S_PIN_STEP    GPIO_NUM_25

// Joint 5 Params
#define J5S_PORT        3
#define J5S_PIN_DIR     GPIO_NUM_14
#define J5S_PIN_STEP    GPIO_NUM_27

// Joint 6 Params
#define J6S_PORT        2
#define J6S_PIN_DIR     GPIO_NUM_13
#define J6S_PIN_STEP    GPIO_NUM_12

// End Effector Params
#define EE_PORT 0

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#endif