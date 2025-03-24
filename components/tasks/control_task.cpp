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
void setMotorAngles(UserCommand* cmd){
    float m1Angle =    cmd->params[0];
    float m2Angle =    cmd->params[1];
    float m3Angle =    cmd->params[2];
    float m4Angle =    cmd->params[3];
    float m5Angle =    cmd->params[4];
    float m6Angle =    cmd->params[5];
    
    uint8_t motorBitMask = 0x00;

    // Disable Task switching until all motor's are setup
    vTaskSuspendAll();

    // Sends each motor it's desired angle, and adds the motor to the bit mask
    if(m1Angle != -1){
        motorBitMask |= J1S_BIT_MASK;
        xQueueSend(desiredAngleQueue[0], &m1Angle, 0);
    }
    if(m2Angle != -1){
        motorBitMask |= J2S_BIT_MASK;
        xQueueSend(desiredAngleQueue[1], &m2Angle, 0);
    }
    if(m3Angle != -1){
        motorBitMask |= J3S_BIT_MASK;
        xQueueSend(desiredAngleQueue[2], &m3Angle, 0);
    }
    if(m4Angle != -1){
        motorBitMask |= J4S_BIT_MASK;
        xQueueSend(desiredAngleQueue[3], &m4Angle, 0);
    }
    if(m5Angle != -1){
        motorBitMask |= J5S_BIT_MASK;
        xQueueSend(desiredAngleQueue[4], &m5Angle, 0);
    }
    if(m6Angle != -1){
        motorBitMask |= J6S_BIT_MASK;
        xQueueSend(desiredAngleQueue[5], &m6Angle, 0);
    }

    // Enable Task switching
    xTaskResumeAll();

    // Wait until all set motors marks they're ready
    xEventGroupWaitBits(motorReady, motorBitMask, pdTRUE, pdTRUE, portMAX_DELAY);

    // Enable Motors
    xEventGroupSetBits(motorEnable, motorBitMask);

    // Clears the 'motorIdle' flag
    xEventGroupClearBits(motorIdle, motorBitMask);

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
                ESP_LOGI(controlTag, "Successfully Started setMotorAngles()");
                setMotorAngles(&cmd);
                break;

            case 11:
                ESP_LOGI(controlTag, "Successfully Started setMotorSpeed()");
                setMotorSpeed(&cmd);
                break;

            case 20:
                ESP_LOGI(controlTag, "Successfully Started sleep()");
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
