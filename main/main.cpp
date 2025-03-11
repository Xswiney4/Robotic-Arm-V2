#include "as5600.h"
#include "pca9548a.h"
#include "stepper.h"

#include <chrono> // For time in ms
#include <thread> // For sleeping
#include <cstdint> // for uint16_t
#include <iostream>
#include <iomanip>

const gpio_num_t STEP_PIN = GPIO_NUM_18;
const gpio_num_t DIR_PIN = GPIO_NUM_5;


extern "C" void app_main(){

    // Object Building
    std::cout << "Creating StepperMotor object..." << std::endl;
    StepperMotor motor(STEP_PIN, DIR_PIN);
    std::cout << "Motor Created." << std::endl;


    gpio_set_level(DIR_PIN, 1);

    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // gpio_set_level(STEP_PIN, 1);
    // std::cout << "Set High" << std::endl;
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // gpio_set_level(STEP_PIN, 0);
    // std::cout << "Set Low" << std::endl;

    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // gpio_set_level(STEP_PIN, 1);
    // std::cout << "Set High" << std::endl;
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // gpio_set_level(STEP_PIN, 0);
    // std::cout << "Set Low" << std::endl;

    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // gpio_set_level(STEP_PIN, 1);
    // std::cout << "Set High" << std::endl;
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // gpio_set_level(STEP_PIN, 0);
    // std::cout << "Set Low" << std::endl;



    motor.setDir(true);

    for(int i = 0; i < 200; i++){
        std::cout << "Step" << std::endl;
        motor.step();
    }
    std::cout << "Changing Direction" << std::endl;
    motor.setDir(false);
    for(int i = 0; i < 200; i++){
        std::cout << "Step" << std::endl;
        motor.step();
    }

    while(true){
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

}