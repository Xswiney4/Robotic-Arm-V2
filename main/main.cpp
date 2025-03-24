
#include "roboticArm.h"

#include <chrono> // For time in ms
#include <thread> // For sleeping
#include <cstdint> // for uint16_t
#include <iostream>
#include <iomanip>
#include "esp_log.h"


extern "C" void app_main(){


    RoboticArm robot;

    robot.setMotorAngles(-1, -1, -1, -1, -1, 0);
    robot.sleep(1000);
    robot.setMotorAngles(-1, -1, -1, -1, -1, 360);
    robot.sleep(1000);
    robot.setMotorAngles(-1, -1, -1, -1, -1, 180);
    robot.sleep(1000);
    robot.setMotorAngles(-1, -1, -1, -1, -1, 270);
    robot.sleep(1000);
    robot.setMotorAngles(-1, -1, -1, -1, -1, 0);

    
    while(true){
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }

}