// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Libraries ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include "stepper.h"
//#include "ets_sys.h"

#include <thread> // For sleeping
#include <chrono> // For time in ms
#include <iostream>

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
}

// Takes a step
void StepperMotor::step(){
    
    // Set pin high, then delay
    if(gpio_set_level(this->stepPin, 1) != ESP_OK){
        std::cout << "GPIO HIGH FAILED" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    // Set pin low, then delay
    if(gpio_set_level(this->stepPin, 0) != ESP_OK){
        std::cout << "GPIO LOW FAILED" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}