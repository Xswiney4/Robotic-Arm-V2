// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Libraries ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include "stepper.h"

#include <thread> // For sleeping
#include <iostream>
#include <chrono>

// ESP/FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Constructor/Destructor ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Constructor: Creates and initializes object
StepperMotor::StepperMotor(gpio_num_t stepPin, gpio_num_t dirPin) : stepPin(stepPin), dirPin(dirPin){
    
    this->setupGPIO();
    this->setDir(true);

}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ GPIO Setup ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void StepperMotor::setupGPIO(){

    gpio_config_t ioConf = {};
    ioConf.intr_type = GPIO_INTR_DISABLE;                                       // Disable interrupts
    ioConf.mode = GPIO_MODE_OUTPUT;                                             // Set as output
    ioConf.pin_bit_mask = (1ULL << this->stepPin) | (1ULL << this->dirPin);     // Selects dir/step pins
    ioConf.pull_down_en = GPIO_PULLDOWN_DISABLE;                                // Disable pull downs
    ioConf.pull_up_en = GPIO_PULLUP_DISABLE;                                    // Disable pull ups

    gpio_config(&ioConf);                                                       // Apply configuration
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Motor Controls ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Sets the direction of the motor
void StepperMotor::setDir(bool clockwise){
    gpio_set_level(this->dirPin, clockwise);
    std::this_thread::sleep_for(std::chrono::microseconds(5));
    
}

// Takes a step
void StepperMotor::step(){
    
    // Set pin high, then delay
    gpio_set_level(this->stepPin, 1);
    std::this_thread::sleep_for(std::chrono::microseconds(4));

    // Set pin low, then delay
    gpio_set_level(this->stepPin, 0);
    std::this_thread::sleep_for(std::chrono::microseconds(4));
}