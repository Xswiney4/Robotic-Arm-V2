// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Libraries ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "control_task.h"

// ESP/FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

// Utils
#include <cstring>
#include <cstdlib>



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
    ESP_LOGI(controlTag, "Set angle: %f", cmd -> params[1]);
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

/*
This function decodes a character array and returns a userCommand structure.
The structure has the following format:
struct userCommand{
    int command;
    double param0;
    double param1;
    double param2;
    double param3;
    double param4;
    double param5;
};

The commands and their given parameters are specified here:
https://docs.google.com/spreadsheets/d/1nJw_hCjyDbuuJXKSwHuH-XOVeJgRIX9mN4UZ95cLr8M/edit?gid=162912065#gid=162912065
*/

UserCommand userCmdParser(char *buffer){
    
    // Buffers
    UserCommand cmd;
    char cmdBuff[64];
    char paramBuff[32];

    // Character index
    int i;
    bool found = false;

    // Search for the first '('
    for(i = 0; i < 64; i++){
        if(buffer[i] == '('){
            cmdBuff[i] = '\0'; // Close up the character array
            i++;               // Move onto the next part in the buffer
            found = true;
            break;
        }
        cmdBuff[i] = buffer[i];
    }
    
    // Error Check
    if(!found){
        ESP_LOGE(controlTag, "Invalid User Command - Could not find '(' within string");
        return {-1};
    }
    // At this point, cmdBuff is our command
    
    // Now we search through the commands to see if it's valid
    int j;
    found = false;
    for(j = 0; j < numCommands; j++){
        if(strcmp(cmdBuff,commands[j].name) == 0){
            // Command Found!
            cmd.name = commands[j].name;
            cmd.commandNum = commands[j].commandNum;
            found = true;
            break;
        }
    }

    // Error Check
    if(!found){
        ESP_LOGE(controlTag, "Invalid User Command - Command not found");
        return {-1};
    }
    
    // Now we need to parse the parameters
    int paramNum;
    int paramFound = false;
    int paramBuffPointer = 0;
    char* paramEndPtr;
    for(paramNum = 0; paramNum < commands[j].numParams; paramNum++){
        paramFound = false;
        for(; i < 64; i++){
            switch (buffer[i]){
                // If we run into a space, skip it
                case ' ': 
                    break;

                // If we run into a comma, that marks the end of a parameter
                case ',':
                    paramBuff[paramBuffPointer] = '\0'; // Close up string
                    cmd.params[paramNum] = std::strtod(paramBuff, &paramEndPtr); // Converts paramBuff into a double and saves
                    if(*paramEndPtr != '\0'){
                        ESP_LOGE(controlTag, "Invalid User Command - Double Conversion Failed");
                        return {-1};
                    }
                    paramBuffPointer = 0;
                    i++; // to move onto the next character
                    paramFound = true;
                    break;

                // If we run into a ')', that marks the end of the function (last parameter)
                case ')':
                    // Checks to ensure we're on the the last parameter
                    if(paramNum != commands[j].numParams - 1){
                        ESP_LOGE(controlTag, "Invalid User Command - Invalid Parameter Count");
                        return {-1};
                    }
                    paramBuff[paramBuffPointer] = '\0'; // Close up string
                    cmd.params[paramNum] = std::strtod(paramBuff, &paramEndPtr); // Converts paramBuff into a double and saves
                    if(*paramEndPtr != '\0'){
                        ESP_LOGE(controlTag, "Invalid User Command - Double Conversion Failed");
                        return {-1};
                    }
                    paramFound = true;
                    break;

                // Otherwise, save the character
                default:
                    paramBuff[paramBuffPointer] = buffer[i];
                    paramBuffPointer++;
            } 
            // If we found the param, we move onto the next param loop
            if (paramFound){break;}
        }
    }

    return cmd;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Task Definitions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void controlTask(void *pvParameter){

    // Task Variables
    QueueSetMemberHandle_t activeQueue;
    UserCommand cmd;
    char cmdBuffer[64];


    while(true){

        // Wait until either of the queues in the set have something
        activeQueue = xQueueSelectFromSet(controlSet, portMAX_DELAY);

        // Decode which queue was activated
        if(activeQueue == userCmdRaw){
            // If it's a raw user command, we parse it and then put it into the cmd variable
            xQueueReceive(userCmdRaw, &cmdBuffer, 0);
            ESP_LOGI(controlTag, "User Command Received: \n    %s", cmdBuffer);
            cmd = userCmdParser(cmdBuffer);
        }
        else if(activeQueue == userCmd){
            // If it's a user command struct, we receive and put it into the cmd variable
            xQueueReceive(userCmd, &cmd, 0);
        }

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
