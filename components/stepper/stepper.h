#ifndef STEPPER_H
#define STEPPER_H

#include "driver/gpio.h"

class StepperMotor{
    private:

        // Private Variables
        gpio_num_t stepPin;
        gpio_num_t dirPin;

        // GPIO Setup
        void setupGPIO();

        
    public:
        // Constructor/Destructor
        StepperMotor(gpio_num_t stepPin, gpio_num_t dirPin);

        // Motor Controls
        void setDir(bool clockwise = false);
        void step();

};

#endif