// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Libraries ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "motorGearboxSystem.h"

// ESP/FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Utils
#include <thread>
#include <chrono>
#include <cmath>

static const char *MotorTag = "Motor Driver";

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Constructor/Destructor ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Constructor: Creates and initializes object
MotorGearboxSystem::MotorGearboxSystem(MotorParams params) : motorParams(params){
    
    // General Setup
    setupGPIO();
    setupAS5600();

    // Sets defaults
    setDir(CLOCKWISE);
    currentDir = CLOCKWISE;
    inputAngle = getRawAngle();
    maxAngle = 360.0f * params.gearRatio;
    
    // Calibration
    calibrate();

}

// Destructor: Creates and initializes object
MotorGearboxSystem::~MotorGearboxSystem(){

}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// ********************************* PRIVATE **************************************
// ~~ Setup ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Sets up GPIO
void MotorGearboxSystem::setupGPIO(){
    gpio_config_t ioConf = {};
    ioConf.intr_type = GPIO_INTR_DISABLE;                                               // Disable interrupts
    ioConf.mode = GPIO_MODE_OUTPUT;                                                     // Set as output
    ioConf.pin_bit_mask = (1ULL << motorParams.stepPin) | (1ULL << motorParams.dirPin); // Selects dir/step pins
    ioConf.pull_down_en = GPIO_PULLDOWN_DISABLE;                                        // Disable pull downs
    ioConf.pull_up_en = GPIO_PULLUP_DISABLE;                                            // Disable pull ups

    gpio_config(&ioConf);                                                               // Apply configuration
}

// Sets up AS5600 object
void MotorGearboxSystem::setupAS5600(){
    as5600.emplace(motorParams.pca9548a, motorParams.pca9548aPort, motorParams.as5600Config);
}

// Runs initial calibration (Synchronizes AS5600 home with it's current zero position)
void MotorGearboxSystem::calibrate(){
    
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// *********************************** PUBLIC **************************************
// ~~ Motor Controls ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Takes a step in currently set direction. Cannot run with less than a 2ns period
void MotorGearboxSystem::step(){
    // Set pin high, then delay
    gpio_set_level(motorParams.stepPin, 1);
    std::this_thread::sleep_for(std::chrono::microseconds(2));

    // Set pin low
    gpio_set_level(motorParams.stepPin, 0);
}

// Sets the direction if it's different, then takes a step
void MotorGearboxSystem::step(bool dir){
    if(currentDir != dir){
        setDir(dir);
    }
    step();
}

// Sets the direction
void MotorGearboxSystem::setDir(bool dir){
    gpio_set_level(motorParams.dirPin, dir);
    currentDir = dir;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Angle Measurements ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Returns the angle straight from AS5600
float MotorGearboxSystem::getRawAngle(){
    return as5600->getAngle();
}

// Returns the output angle after adjusting due to gearbox
float MotorGearboxSystem::getAngle(){
    
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Servo Controls through Setup ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Sets the target angle parameter
void MotorGearboxSystem::setupTargetAngle(float angle){
    targetParams.targetAngle = angle;
}

// Calculates the target frequency and sets the parameter
void MotorGearboxSystem::setupTargetSpeed(float speed){
    targetParams.xFrequency = pdMS_TO_TICKS(roundf(1800.0f / speed));
}

// Sets the target step frequency parameter
void MotorGearboxSystem::setupTargetFrequency(TickType_t xFrequency){
    targetParams.xFrequency = xFrequency;
}

// Resets the target parameter to default
void MotorGearboxSystem::resetTarget(){
    targetParams = {-1.0f, (TickType_t)-1};
}

// Starts moving the motor to it's target based on the targetParams
void MotorGearboxSystem::start(){

    // Error Check
    if(targetParams.targetAngle == -1.0f){
        // Target angle not set
        ESP_LOGE(MotorTag, "Target angle not set, returning...");
        return;
    }
    else if(targetParams.xFrequency == (TickType_t)-1){
        // Target xFrequency not set
        ESP_LOGE(MotorTag, "Target xFrequency not set, returning...");
        return;
    }

    // START LOGIC HERE...

    resetTarget();
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Auto-Start Controls ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void MotorGearboxSystem::setTargetAngle(float angle, float speed){
    setupTargetAngle(angle);
    setupTargetSpeed(speed);
    start();
}

void MotorGearboxSystem::setTargetAngle(float angle, TickType_t xFrequency){
    setupTargetAngle(angle);
    setupTargetFrequency(xFrequency);
    start();
}