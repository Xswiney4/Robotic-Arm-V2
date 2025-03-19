#include "config.h"
#include "as5600.h"
#include "pca9548a.h"
#include "stepper.h"
#include "communication_tasks.h"
#include "control_task.h"

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


    initCommunications();

    char myCmd[64] = "setEndSpeed(214, 1243, 2131, 213 ,123)";

    UserCommand myUserCmd = userCmdDecoder(myCmd);

    std::cout << "Command Num: " << myUserCmd.commandNum << std::endl;
    std::cout << "Name:        " << myUserCmd.name << std::endl;
    std::cout << "Param 1      " << myUserCmd.params[0] << std::endl;
    std::cout << "Param 2      " << myUserCmd.params[1] << std::endl;
    std::cout << "Param 3      " << myUserCmd.params[2] << std::endl;
    std::cout << "Param 4      " << myUserCmd.params[3] << std::endl;
    std::cout << "Param 5      " << myUserCmd.params[4] << std::endl;
    std::cout << "Param 6      " << myUserCmd.params[5] << std::endl;

    

    while(true){
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

}