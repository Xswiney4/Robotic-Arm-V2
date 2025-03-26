// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Libraries ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "control_task.h"
#include "config.h"

// ESP/FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

// Utils



// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Definitions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

static const char *controlTag = "Control Task";

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Initializations ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ User Commands ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/*
Sets the endpoint given a position (x, y, z) and orientation (pitch, yaw, roll) of the end effector
The control structure is as follows:
- UserCommand is sent over to Kinematics Solver to be solved
- Wait until all motors are ready AND Kinematic Solver flags 'solved'
- Enable all motors
- Break
*/
void setEnd(UserCommand* cmd){
    // double x =      cmd->params[0];
    // double y =      cmd->params[1];
    // double z =      cmd->params[2];
    // double pitch =  cmd->params[3];
    // double yaw =    cmd->params[4];
    // double roll =   cmd->params[5];
}

/*
Sets the endpoint speed in m/s
The structure is as follows:
- 
*/
void setEndSpeed(UserCommand* cmd){
    // double endSpeed = cmd->params[0];
}

/*
Sets the angle of a given motor in degrees
The structure is as follows:
- Angle is sent over to motor
- Wait until given motor is ready
- Enable given motor
- Break
*/
void setMotorAngles(UserCommand* cmd, MotorParams* motorParams){
    // float m1Angle =    cmd->params[0];
    // float m2Angle =    cmd->params[1];
    // float m3Angle =    cmd->params[2];
    // float m4Angle =    cmd->params[3];
    // float m5Angle =    cmd->params[4];
    // float m6Angle =    cmd->params[5];
    
    uint8_t motorBitMask = 0x00;

    // Loop going through each parameter input
    for(int i = 0; i < 6; i++){
        if(cmd->params[i] != -1){
            // If parameter is valid

            // Sends each motor it's desired angle, and adds the motor to the bit mask
            motorBitMask |= motorParams[i].eventGroupBit;
            
            motorParams[i].targetAngle = cmd->params[i];

        }
    }

    // Clears the 'motorIdle' flag
    xEventGroupClearBits(motorIdle, motorBitMask);
    
    // Enable Motors
    xEventGroupSetBits(motorEnable, motorBitMask);

    // Wait until all motor's are idle before continuing
    xEventGroupWaitBits(motorIdle, motorBitMask, pdFALSE, pdTRUE, portMAX_DELAY);

}

/*
Sets the given motor speed in m/s
The structure is as follows:
- UserCommand is sent over to Kinematics Solver to be solved
- Wait until given motor is ready AND Kinematic Solver flags 'solved'
- Break
*/
void setMotorSpeed(UserCommand* cmd){
    // int motor =     (int)cmd->params[0];
    // double speed =       cmd->params[1];
}

// Puts the controller to sleep for a given time in ms
void sleep(UserCommand* cmd){
    ESP_LOGD(controlTag, "Delaying control task for %dms", (int)cmd->params[0]);
    vTaskDelay(pdMS_TO_TICKS((int)cmd->params[0]));
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Handlers ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Task Definitions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void controlTask(void *pvParameter){

    // Cast Motor Params
    MotorParams *motorParams = (MotorParams *)pvParameter;


    // Task Variables
    UserCommand cmd;

    while(true){

        // Wait until user command struct is recieved
        xQueueReceive(controlCmd, &cmd, portMAX_DELAY);

        // Now we decode the command struct
        switch (cmd.commandNum){
            // Invalid Command
            case -1:
                ESP_LOGE(controlTag, "Invalid command");
                break;
                
            case 0:
                ESP_LOGD(controlTag, "Successfully Started setEnd()");
                setEnd(&cmd);
                break;
            
            case 1:
                ESP_LOGD(controlTag, "Successfully Started setEndSpeed()");
                setEndSpeed(&cmd);
                break;
            
            case 10:
                ESP_LOGD(controlTag, "Successfully Started setMotorAngles()");
                setMotorAngles(&cmd, motorParams);
                break;

            case 11:
                ESP_LOGD(controlTag, "Successfully Started setMotorSpeed()");
                setMotorSpeed(&cmd);
                break;

            case 20:
                ESP_LOGD(controlTag, "Successfully Started sleep()");
                sleep(&cmd);
                break;

            default:
                ESP_LOGE(controlTag, "Unknown cmd.commandNum");
                break;
        }



    }
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Other Utils ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
