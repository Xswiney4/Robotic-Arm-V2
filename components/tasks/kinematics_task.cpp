// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Libraries ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "robotic_definitions.h"
#include "kinematics_task.h"

// ESP/FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Definitions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

static const char *kinematicsTag = "Kinematics Task";

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ User Commands ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Converts a position and orientation into joint angles, and then sends them to the control task
void setEndKinCalculation(UserCommand* cmd){

    // Variable Declaration
    // float x = cmd->params[0];
    // float y = cmd->params[1];
    // float z = cmd->params[2];
    // float pitch = cmd->params[3];
    // float yaw = cmd->params[4];
    // float roll = cmd->params[5];


}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Task Definition ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


void kinematicsTask(void *pvParameter){

    // Task Variables
    UserCommand cmd;

    while(true){

        // Wait until user command struct is recieved
        xQueueReceive(kinematicsCmd, &cmd, portMAX_DELAY);

        // Now we decode the command struct
        switch (cmd.commandNum){
            // Invalid Command
            case -1:
                ESP_LOGE(kinematicsTag, "Invalid command");
                break;
                
            case 0:
                ESP_LOGI(kinematicsTag, "Successfully Started setEnd()");
                setEndKinCalculation(&cmd);
                break;

            default:
                ESP_LOGD(kinematicsTag, "No processing required, throwing out command");
                break;
        }

    }

}