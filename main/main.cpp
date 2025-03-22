
#include "roboticArm.h"

#include <chrono> // For time in ms
#include <thread> // For sleeping
#include <cstdint> // for uint16_t
#include <iostream>
#include <iomanip>
#include "esp_log.h"


extern "C" void app_main(){


    RoboticArm robot;

    
    while(true){
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }

}