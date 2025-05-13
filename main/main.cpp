
#include "roboticArm.h"
#include "config.h"

#include <chrono> // For time in ms
#include <thread> // For sleeping
#include <cstdint> // for uint16_t
#include <iostream>
#include <iomanip>
#include "esp_log.h"
#include <cmath>
#include <random>

#define SLEEP_TIME 500
#define NUMTRIALS 20

extern "C" void app_main(){

    // Create a random number generator
    std::random_device rd;
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine
    std::uniform_real_distribution<float> dist(0.0f, 360.0f);

    RoboticArm robot;

    for(int i = 0; i < NUMTRIALS; i++){
        float rand = dist(gen);
        robot.setAngles(-1.0f, rand, -1.0f, -1.0f, -1.0f, rand);
        robot.sleep(SLEEP_TIME);

    }
    
    while(true){
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }

}