// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Libraries ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "control_task.h"

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
void setMotorAngle(UserCommand* cmd){
    int motor =     (int)cmd->params[0];
    uint8_t motorBitMask = (1 << (motor - 1));

    // Send angle to the motor
    ESP_LOGD(controlTag, "Set angle: %f", cmd -> params[1]);
    xQueueSend(desiredAngleQueue[motor - 1], &cmd -> params[1], 0);

    // Wait until the set motor marks it's ready
    xEventGroupWaitBits(motorReady, motorBitMask, pdTRUE, pdFALSE, portMAX_DELAY);

    // Enable Motor
    xEventGroupSetBits(motorEnable, motorBitMask);

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

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Handlers ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Task Definitions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void controlTask(void *pvParameter){

    // Task Variables
    UserCommand cmd;

    while(true){

        // Wait until user command struct is recieved
        xQueueReceive(controlCmd, &cmd, portMAX_DELAY);

        // Now we decode the command struct
        switch (cmd.commandNum){
            // Invalid Command
            case -1:
                break;
                
            case 0:
                ESP_LOGI(controlTag, "Successfully Started setEnd()");
                setEnd(&cmd);
                break;
            
            case 1:
                ESP_LOGI(controlTag, "Successfully Started setEndSpeed()");
                setEndSpeed(&cmd);
                break;
            
            case 10:
                ESP_LOGI(controlTag, "Successfully Started setMotorAngle()");
                setMotorAngle(&cmd);
                break;

            case 11:
                ESP_LOGI(controlTag, "Successfully Started setMotorSpeed()");
                setMotorSpeed(&cmd);
                break;

            default:
                ESP_LOGE(controlTag, "Unknown cmd.commandNum");
                break;
        }



    }
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Other Utils ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
