// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Libraries ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "motorModule.h"

// ESP/FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Utils
#include <thread>
#include <chrono>
#include <cmath>

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Constructor/Destructor ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Constructor: Creates and initializes object
MotorModule::MotorModule(MotorParams params) : motorParams(params){
    
    // General Setup
    setupGPIO();
    setupAS5600();

    // Sets defaults
    setDir(CLOCKWISE);
    currentAngle = as5600->getAngle();
    
    // Calibration
    calibrate();

}

// Destructor: Creates and initializes object
MotorModule::~MotorModule(){

}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// ********************************* PRIVATE **************************************
// ~~ Setup ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Sets up GPIO
void MotorModule::setupGPIO(){
    gpio_config_t ioConf = {};
    ioConf.intr_type = GPIO_INTR_DISABLE;                                               // Disable interrupts
    ioConf.mode = GPIO_MODE_OUTPUT;                                                     // Set as output
    ioConf.pin_bit_mask = (1ULL << motorParams.stepPin) | (1ULL << motorParams.dirPin); // Selects dir/step pins
    ioConf.pull_down_en = GPIO_PULLDOWN_DISABLE;                                        // Disable pull downs
    ioConf.pull_up_en = GPIO_PULLUP_DISABLE;                                            // Disable pull ups

    gpio_config(&ioConf);                                                               // Apply configuration
}

// Sets up AS5600 object
void MotorModule::setupAS5600(){
    as5600.emplace(motorParams.pca9548a, motorParams.pca9548aPort, motorParams.as5600Config);
}

// Runs initial calibration (Synchronizes AS5600 home with it's current zero position)
void MotorModule::calibrate(){
    return;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Servo Controls through Setup ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


void MotorModule::setupTargetSteps(int numSteps){
    targetParams.numSteps = numSteps;
    ESP_LOGD(pcTaskGetName(NULL), "Set numSteps to: %d", numSteps);
}

// Sets the target angle parameter
void MotorModule::setupTargetAngle(float targetAngle){

    // Difference the input motor needs to move in order for the output angle to reach the target
    float inputDiff = motorParams.gearRatio * (targetAngle - currentAngle);

    // Calculate the number of steps needed
    targetParams.numSteps = (int)roundf(inputDiff * motorParams.microstepping / 1.8f);

    // Debug log
    ESP_LOGD(pcTaskGetName(NULL), "Calculated %d steps required to go from %.2f to %.2f", targetParams.numSteps, currentAngle, targetAngle);
}

// Calculates the target frequency and sets the parameter
void MotorModule::setupTargetSpeed(float speed){

    // Calculates the step time in ms
    float stepPeriodMs = 1800.0f / (speed * motorParams.gearRatio);

    // If the step time is not a whole number
    if(stepPeriodMs != (int)(stepPeriodMs)){
        stepPeriodMs = roundf(stepPeriodMs);
        ESP_LOGE(pcTaskGetName(NULL), "Exact speed not achievable... rounding to %dms step time", (int)stepPeriodMs);
    }

    // If the step time is less than the FreeRTOS tick rate, it's theoretically impossible to achieve
    if(stepPeriodMs < configTICK_RATE_HZ / 1000.0f){
        ESP_LOGE(pcTaskGetName(NULL), "Speed too fast, step period set to %dms", (int)(configTICK_RATE_HZ / 1000.0f));
        stepPeriodMs = configTICK_RATE_HZ / 1000.0f;
    }

    targetParams.xFrequency = pdMS_TO_TICKS(stepPeriodMs);

    // Debug log
    ESP_LOGD(pcTaskGetName(NULL), "Converted %.2f deg/sec to a an xFrequency of %lu", speed, targetParams.xFrequency);
}

// Sets the target step frequency parameter
void MotorModule::setupTargetFrequency(TickType_t xFrequency){
    targetParams.xFrequency = xFrequency;
    ESP_LOGD(pcTaskGetName(NULL), "Set xFrequency to: %lu", targetParams.xFrequency);
}

// Starts moving the motor to it's target based on the targetParams
void MotorModule::moveToTarget(){

    // Check if we're already at the target
    if(targetParams.numSteps == 0){
        ESP_LOGD(pcTaskGetName(NULL), "Already at the target angle");
        return;
    }

    // Measure the start of the movement (This is used for validation)
    startAngle = updateAngle();
    ESP_LOGD(pcTaskGetName(NULL), "Starting angle is: %.2f", startAngle);

    // First we set the direction, based on the sign of numSteps
    if(targetParams.numSteps < 0.0f){
        setDir(CLOCKWISE);
        targetParams.numSteps = fabs(targetParams.numSteps);
    }
    else{
        setDir(COUNTERCLOCKWISE);
    }
    
    // Delay after setting direction
    std::this_thread::sleep_for(std::chrono::microseconds(2));

    // RTOS Tick Setup
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // Continues moving the amount of steps required
    while(true){

        // Critical section ensures validation doesn't interrupt this
        taskENTER_CRITICAL(&stepMux);

        step(); // Take a step
        targetParams.numSteps--; // Decrement numSteps

        // Exit critical section
        taskEXIT_CRITICAL(&stepMux);
        
        // Check for exit
        if(targetParams.numSteps == 0){
            ESP_LOGD(pcTaskGetName(NULL), "NumSteps achieved");
            break;
        }

        
        // Block for delay
        vTaskDelayUntil(&xLastWakeTime, targetParams.xFrequency);

        // Updates the angle before it passes 180 degrees
        if(targetParams.numSteps % (int)(80.0f * motorParams.microstepping) == 0){
            updateAngle();
        }
    }
    ESP_LOGD(pcTaskGetName(NULL), "End Angle: %.2f", updateAngle());

}
// *********************************** PUBLIC **************************************
// ~~ Motor Controls ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Takes a step in currently set direction. Cannot run with less than a 2us period
void MotorModule::step(){
    // Set pin high, then delay
    gpio_set_level(motorParams.stepPin, 1);
    std::this_thread::sleep_for(std::chrono::microseconds(2));

    // Set pin low
    gpio_set_level(motorParams.stepPin, 0);
}

// Sets the direction if it's different, then takes a step
void MotorModule::step(bool dir){
    if(currentDir != dir){
        setDir(dir);
        std::this_thread::sleep_for(std::chrono::microseconds(2));
    }
    step();
}

// Sets the direction
void MotorModule::setDir(bool dir){
    gpio_set_level(motorParams.dirPin, dir);
    currentDir = dir;
    ESP_LOGD(pcTaskGetName(NULL), "Direction set to: %s", dir == CLOCKWISE ? "Clockwise" : "Counterclockwise");
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Angle Measurements ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Returns the angle straight from AS5600
float MotorModule::updateAngle(){

    // Get new angle value
    float rawAngle = as5600->getAngle();
    float difference = currentAngle - rawAngle;

    // Update the last angle
    lastCurrentAngle = currentAngle;

    // Check for a crossover the 0/360 threshold
    if(difference > 180.0f){
        // Crossed 360 -> 0 (Clockwise)
        ESP_LOGD(pcTaskGetName(NULL), "Angle update clamped to 360");
        currentAngle = 360.0f;
    }
    else if(difference  < -180.0f){
        // Crossed 0 -> 360 (Counterclockwise)
        ESP_LOGD(pcTaskGetName(NULL), "Angle update clamped to 0");
        currentAngle = 0.0f;
    }
    else{
        // Normal Update
        currentAngle = rawAngle;
    }

    return currentAngle;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Validation ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Checks to ensure the current step matches the input angle, and adjusts the step count if necessary
bool MotorModule::validateCurrentStep(){
    return true;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Auto-Start Controls ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Moves the stepper motor to it's target (Slowest)
void MotorModule::setAngle(float angle, float speed){
    setupTargetAngle(angle);
    setupTargetSpeed(speed);
    moveToTarget();
}

// Moves the stepper motor to it's target (Slow)
void MotorModule::setAngle(float angle, TickType_t xFrequency){
    setupTargetAngle(angle);
    setupTargetFrequency(xFrequency);
    moveToTarget();
}

// Moves the stepper motor to it's target (Slow)
void MotorModule::setAngle(int numSteps, float speed){
    setupTargetSteps(numSteps);
    setupTargetSpeed(speed);
    moveToTarget();
}

// Moves the stepper motor to it's target (Fastest)
void MotorModule::setAngle(int numSteps, TickType_t xFrequency){
    setupTargetSteps(numSteps);
    setupTargetFrequency(xFrequency);
    moveToTarget();
}