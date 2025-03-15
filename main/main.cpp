#include "config.h"
#include "as5600.h"
#include "pca9548a.h"
#include "stepper.h"
#include "communication_tasks.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/ledc.h"

#include <chrono> // For time in ms
#include <thread> // For sleeping
#include <cstdint> // for uint16_t
#include <iostream>
#include <iomanip>

const gpio_num_t STEP_PIN = J6S_PIN_STEP;
const gpio_num_t DIR_PIN = J6S_PIN_DIR;


extern "C" void app_main(){


    xTaskCreate(initBluetooth, "bluetooth_task", 4096, NULL, 5, NULL);


    while(true){
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

}