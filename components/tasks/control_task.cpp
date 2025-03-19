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

char cmdBuffer[64];
static const char *controlTag = "Control Task";

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Initializations ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void initControl(){

}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ User Commands ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/*
Sets the endpoint given a position (x, y, z) and orientation (pitch, yaw, roll) of the end effector
The control structure is as follows:
- UserCommand is sent over to Kinematics Solver to be solved
- Wait until all motors are idle AND Kinematic Solver flags 'solved'
- Enable all motors
- Break
*/
void setEnd(double x, double y, double z, double pitch, double yaw, double roll){

}

/*
Sets the endpoint speed in m/s
The structure is as follows:
- 
*/
void setEndSpeed(double speed){
    
}

/*
Sets the angle of a given motor in degrees
The structure is as follows:
- UserCommand is sent over to Kinematics Solver to be solved
- Wait until given motor is idle AND Kinematic Solver flags 'solved'
- Enable given motor
- Break
*/
void setMotorAngle(int motor, double angle){

}

/*
Sets the given motor speed in m/s
The structure is as follows:
- UserCommand is sent over to Kinematics Solver to be solved
- Wait until given motor is idle AND Kinematic Solver flags 'solved'
- Break
*/
void setMotorSpeed(int motor, double speed){
    
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

UserCommand userCmdDecoder(char *buffer){
    
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
    UserCommand cmd;
    while(true){

        // Waits until a User Command has been received
        xQueueReceive(userCmdRaw, &cmdBuffer, portMAX_DELAY);
        ESP_LOGI(controlTag, "User Command Input: \n %s", cmdBuffer);

        // Once received, we decode the message, and then respond accordingly
        ESP_LOGI(controlTag, "Decoding Message...");
        cmd = userCmdDecoder(cmdBuffer);

        switch (cmd.commandNum){
            // Invalid Command
            case -1:
                break;
                
            case 0:
                ESP_LOGI(controlTag, "Successfully Started setEnd()");
                setEnd(cmd.params[0], cmd.params[1], cmd.params[2], cmd.params[3], cmd.params[4], cmd.params[5]);
                break;
            
            case 1:
                ESP_LOGI(controlTag, "Successfully Started setEndSpeed()");
                setEndSpeed(cmd.params[0]);
                break;
            
            case 10:
                ESP_LOGI(controlTag, "Successfully Started setMotorAngle()");
                setMotorAngle(cmd.params[0], cmd.params[1]);
                break;

            case 11:
                ESP_LOGI(controlTag, "Successfully Started setMotorSpeed()");
                setMotorSpeed(cmd.params[0], cmd.params[1]);
                break;

            default:
                ESP_LOGE(controlTag, "Unknown cmd.commandNum");
                break;
        }



    }
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Other Utils ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
