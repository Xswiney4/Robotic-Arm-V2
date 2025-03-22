// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Libraries ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include "robotic_definitions.h"
#include "config.h"
#include "roboticArm.h"

// Component Definitions
#include "as5600.h"
#include "stepper.h"
// #include "pca9548a.h" **Defined in header**

// Tasks Definitions
#include "communication_tasks.h"
#include "control_task.h"
#include "kinematics_task.h"
#include "motor_task.h"

// ESP/FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Tags
static const char *roboticArmInitTag = "Robotic Arm Initializer";
static const char *motorInitTag = "Motor Initializer";

// ~~~ FreeRTOS ~~~
// Queues
QueueHandle_t userCmdRaw;  // Queue for raw user commands (Character Array)
QueueHandle_t userCmd;  // Queue for user commands (UserCommand Struct)

QueueHandle_t j1sDesiredAngleQueue;  // Queue for stepper motor angle
QueueHandle_t j1sParamsQueue;  // Queue for stepper motor params
QueueHandle_t j2sDesiredAngleQueue;  // Queue for stepper motor angle
QueueHandle_t j2sParamsQueue;  // Queue for stepper motor params
QueueHandle_t j3sDesiredAngleQueue;  // Queue for stepper motor angle
QueueHandle_t j3sParamsQueue;  // Queue for stepper motor params
QueueHandle_t j4sDesiredAngleQueue;  // Queue for stepper motor angle
QueueHandle_t j4sParamsQueue;  // Queue for stepper motor params
QueueHandle_t j5sDesiredAngleQueue;  // Queue for stepper motor angle
QueueHandle_t j5sParamsQueue;  // Queue for stepper motor params
QueueHandle_t j6sDesiredAngleQueue;  // Queue for stepper motor angle
QueueHandle_t j6sParamsQueue;  // Queue for stepper motor params

// Task Notification
TaskHandle_t KinematicsSolved; // Flags if Kinematics Solver is idle

// Event Groups
EventGroupHandle_t motorEnable; // Enables Motors
EventGroupHandle_t motorIdle;   // Flags if motor is idle

// ~~~~~~~~~~~~~~~


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Constructor/Destructor ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

RoboticArm::RoboticArm(){

    // PCA9548A Creation
    Pca9548aParams pca9548aParams = {I2C_SCL_PIN, I2C_SDA_PIN, PCA9548A_SLAVE_ADDR, I2C_FREQ, I2C_NUM_0};
    PCA9548A pca9548a(pca9548aParams);

    this->pca9548a = &pca9548a;

    initAll();
}

RoboticArm::~RoboticArm(){

}

// ********************************* PRIVATE **************************************

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Initializations ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Runs all initializations
bool RoboticArm::initAll(){
    
    // RTOS Comms
    if(this->initRTOSComms()){
        ESP_LOGE(roboticArmInitTag, "Error initializing RTOS Communications");
        return true;
    }
    else{
        ESP_LOGI(roboticArmInitTag, "RTOS Communications successfully initialized");
    }

    // Communications Task
    if(this->initCommunications()){
        ESP_LOGE(roboticArmInitTag, "Error initializing Communications Task");
        return true;
    }
    else{
        ESP_LOGI(roboticArmInitTag, "Communications Task successfully initialized");
    }

    // Controls Task
    if(this->initControl()){
        ESP_LOGE(roboticArmInitTag, "Error initializing Controls Task");
        return true;
    }
    else{
        ESP_LOGI(roboticArmInitTag, "Controls Task successfully initialized");
    }

    // Kinematics Calculation Task
    if(this->initKinematics()){
        ESP_LOGE(roboticArmInitTag, "Error initializing Kinematics Calculation Task");
        return true;
    }
    else{
        ESP_LOGI(roboticArmInitTag, "Kinematics Calculation Task successfully initialized");
    }

    // Motor Tasks
    if(this->initAllMotors()){
        ESP_LOGE(roboticArmInitTag, "Error initializing Motor Tasks");
        return true;
    }
    else{
        ESP_LOGI(roboticArmInitTag, "Motor Tasks successfully initialized");
    }

    // Initialization Successful
    return false;

}

// Configures FreeRTOS queues/semaphores/event groups
bool RoboticArm::initRTOSComms(){

    try{
        // Queues
        userCmdRaw = xQueueCreate(10, sizeof(char) * 64);
        userCmd = xQueueCreate(10, sizeof(UserCommand));

        j1sDesiredAngleQueue = xQueueCreate(MOTOR_QUEUE_SIZE, sizeof(float));
        j1sParamsQueue = xQueueCreate(MOTOR_QUEUE_SIZE, sizeof(MotorParams));
        j2sDesiredAngleQueue = xQueueCreate(MOTOR_QUEUE_SIZE, sizeof(float));
        j2sParamsQueue = xQueueCreate(MOTOR_QUEUE_SIZE, sizeof(MotorParams));
        j3sDesiredAngleQueue = xQueueCreate(MOTOR_QUEUE_SIZE, sizeof(float));
        j3sParamsQueue = xQueueCreate(MOTOR_QUEUE_SIZE, sizeof(MotorParams));
        j4sDesiredAngleQueue = xQueueCreate(MOTOR_QUEUE_SIZE, sizeof(float));
        j4sParamsQueue = xQueueCreate(MOTOR_QUEUE_SIZE, sizeof(MotorParams));
        j5sDesiredAngleQueue = xQueueCreate(MOTOR_QUEUE_SIZE, sizeof(float));
        j5sParamsQueue = xQueueCreate(MOTOR_QUEUE_SIZE, sizeof(MotorParams));
        j6sDesiredAngleQueue = xQueueCreate(MOTOR_QUEUE_SIZE, sizeof(float));
        j6sParamsQueue = xQueueCreate(MOTOR_QUEUE_SIZE, sizeof(MotorParams));

        // Task Notification


        // Event Groups
        motorEnable = xEventGroupCreate();
        motorIdle = xEventGroupCreate();

        return false;
    }
    catch(...){
        return true;
    }


}

// Initializes communcations task
bool RoboticArm::initCommunications(){
    return false;
}

// Initializes central control task
bool RoboticArm::initControl(){
    return false;
}

// Initializes kinematics calculations task
bool RoboticArm::initKinematics(){
    return false;
}

// Initializes all motor tasks
bool RoboticArm::initAllMotors(){

    // Motor 1 Init
    if(this->initMotor1()){
        //return true;
    }

    // Motor 2 Init
    if(this->initMotor2()){
        //return true;
    }

    // Motor 3 Init
    if(this->initMotor3()){
        //return true;
    }

    // Motor 4 Init
    if(this->initMotor4()){
        //return true;
    }

    // Motor 5 Init
    if(this->initMotor5()){
        //return true;
    }

    // Motor 6 Init
    if(this->initMotor6()){
        //return true;
    }

    // Initialization Successful
    return false;

}

// Initializes motor task 1
bool RoboticArm::initMotor1(){

    // Motor 1
    try{
        AS5600 j1sAS5600(this->pca9548a, J1S_PORT, CONF);
        StepperMotor j1sMotor(J1S_PIN_STEP, J1S_PIN_DIR);
        MotorParams j1sParams = {&j1sMotor, &j1sAS5600, STEPPER_SPEED, BIT0, j1sDesiredAngleQueue, j1sParamsQueue};
        xTaskCreate(motorTask, J1S_TASK_NAME, 2048, &j1sParams, 1, NULL);
        ESP_LOGI(motorInitTag, "Motor 1 successfully initialized");
        return false;
    }
    catch(...){
        ESP_LOGE(motorInitTag, "Error initializing Motor 1");
        return true;
    }

}

// Initializes motor task 2
bool RoboticArm::initMotor2(){

    // Motor 2
    try{
        AS5600 j2sAS5600(this->pca9548a, J2S_PORT, CONF);
        StepperMotor j2sMotor(J2S_PIN_STEP, J2S_PIN_DIR);
        MotorParams j2sParams = {&j2sMotor, &j2sAS5600, STEPPER_SPEED, BIT1, j2sDesiredAngleQueue, j2sParamsQueue};
        xTaskCreate(motorTask, J2S_TASK_NAME, 2048, &j2sParams, 1, NULL);
        ESP_LOGI(motorInitTag, "Motor 2 successfully initialized");
        return false;
    }
    catch(...){
        ESP_LOGE(motorInitTag, "Error initializing Motor 2");
        return true;
    }

}

// Initializes motor task 3
bool RoboticArm::initMotor3(){

    // Motor 3
    try{
        AS5600 j3sAS5600(this->pca9548a, J3S_PORT, CONF);
        StepperMotor j3sMotor(J3S_PIN_STEP, J3S_PIN_DIR);
        MotorParams j3sParams = {&j3sMotor, &j3sAS5600, STEPPER_SPEED, BIT2, j3sDesiredAngleQueue, j3sParamsQueue};
        xTaskCreate(motorTask, J3S_TASK_NAME, 2048, &j3sParams, 1, NULL);
        ESP_LOGI(motorInitTag, "Motor 3 successfully initialized");
        return false;
    }
    catch(...){
        ESP_LOGE(motorInitTag, "Error initializing Motor 3");
        return true;
    }

}

// Initializes motor task 4
bool RoboticArm::initMotor4(){

    // Motor 4
    try{
        AS5600 j4sAS5600(this->pca9548a, J4S_PORT, CONF);
        StepperMotor j4sMotor(J4S_PIN_STEP, J4S_PIN_DIR);
        MotorParams j4sParams = {&j4sMotor, &j4sAS5600, STEPPER_SPEED, BIT3, j4sDesiredAngleQueue, j4sParamsQueue};
        xTaskCreate(motorTask, J4S_TASK_NAME, 2048, &j4sParams, 1, NULL);
        ESP_LOGI(motorInitTag, "Motor 4 successfully initialized");
        return false;
    }
    catch(...){
        ESP_LOGE(motorInitTag, "Error initializing Motor 4");
        return true;
    }

}

// Initializes motor task 5
bool RoboticArm::initMotor5(){

    // Motor 5
    try{
        AS5600 j5sAS5600(this->pca9548a, J5S_PORT, CONF);
        StepperMotor j5sMotor(J5S_PIN_STEP, J5S_PIN_DIR);
        MotorParams j5sParams = {&j5sMotor, &j5sAS5600, STEPPER_SPEED, BIT4, j5sDesiredAngleQueue, j5sParamsQueue};
        xTaskCreate(motorTask, J5S_TASK_NAME, 2048, &j5sParams, 1, NULL);
        ESP_LOGI(motorInitTag, "Motor 5 successfully initialized");
        return false;
    }
    catch(...){
        ESP_LOGE(motorInitTag, "Error initializing Motor 5");
        return true;
    }

}

// Initializes motor task 6
bool RoboticArm::initMotor6(){

    // Motor 6
    try{
        AS5600 j6sAS5600(this->pca9548a, J6S_PORT, CONF);
        StepperMotor j6sMotor(J6S_PIN_STEP, J6S_PIN_DIR);
        MotorParams j6sParams = {&j6sMotor, &j6sAS5600, STEPPER_SPEED, BIT5, j6sDesiredAngleQueue, j6sParamsQueue};
        xTaskCreate(motorTask, J6S_TASK_NAME, 2048, &j6sParams, 1, NULL);
        ESP_LOGI(motorInitTag, "Motor 6 successfully initialized");
        return false;
    }
    catch(...){
        ESP_LOGE(motorInitTag, "Error initializing Motor 6");
        return true;
    }

}

// *********************************** PUBLIC **************************************