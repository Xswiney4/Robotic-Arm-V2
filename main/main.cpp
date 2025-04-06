
#include "roboticArm.h"
#include "config.h"

#include <chrono> // For time in ms
#include <thread> // For sleeping
#include <cstdint> // for uint16_t
#include <iostream>
#include <iomanip>
#include "esp_log.h"
#include <cmath>

#define TOTAL_STEPS 1000

extern "C" void app_main(){

    RoboticArm robot;
    
    // TickType_t xFrequency = pdMS_TO_TICKS(1800.0f / STEPPER_SPEED);

    // // RTOS Tick Setup
    // TickType_t xLastWakeTime = xTaskGetTickCount();

    // float currentAngle = robot.motorParams[5].as5600->getAngle();

    // robot.motorParams[5].stepper->setDir(true); // Take a step
    // //robot.motorParams[1].stepper->setDir(true); // Take a step


    // // Start timer
    // auto start = std::chrono::high_resolution_clock::now();

    // // Take steps until we reach the target
    // for(int i = 0; i < TOTAL_STEPS; i++){

    //     vTaskDelayUntil(&xLastWakeTime, xFrequency); // Block until delay

    //     robot.motorParams[5].stepper->step(); // Take a step
    //     robot.motorParams[1].stepper->step(); // Take a step

    //     //currentAngle = robot.motorParams[5].as5600->getAngle();
    //     //currentAngle = robot.motorParams[1].as5600->getAngle();

    // }

    // // Stop timer
    // auto end = std::chrono::high_resolution_clock::now();

    // // Calculate elapsed time in seconds
    // std::chrono::duration<float> elapsed_time = end - start;

    // float recordedSpeed = (1.8f * TOTAL_STEPS) / elapsed_time.count();
    // std::cout << "Recorded Speed: " << recordedSpeed << " degrees/second" << std::endl;

    robot.setMotorAngles(-1, 360, -1, -1, -1, 0);
    robot.sleep(1000);
    robot.setMotorAngles(-1, 0, -1, -1, -1, 360);
    robot.sleep(1000);
    robot.setMotorAngles(-1, 180, -1, -1, -1, 180);
    robot.sleep(1000);
    robot.setMotorAngles(-1, 90, -1, -1, -1, 270);
    robot.sleep(1000);
    robot.setMotorAngles(-1, 360, -1, -1, -1, 0);

    
    while(true){
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }

}