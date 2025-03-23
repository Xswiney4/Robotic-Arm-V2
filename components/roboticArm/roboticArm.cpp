// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Libraries ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
// #include "motor_task.h"

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
QueueHandle_t controlCmd;       // Queue for user commands      (UserCommand Struct)
QueueHandle_t kinematicsCmd; // Queue for kinematics task    (UserCommand Struct)

QueueHandle_t desiredAngleQueue[6]; // Queue for stepper motor angles
QueueHandle_t paramsQueue[6];       // Queue for stepper motor params

// Task Notification
TaskHandle_t KinematicsSolved; // Flags if Kinematics Solver is idle

// Event Groups
EventGroupHandle_t motorEnable; // Enables Motors
EventGroupHandle_t motorIdle;   // Flags if motor is idle
EventGroupHandle_t motorReady;  // Signals motor is ready

// ~~~~~~~~~~~~~~~


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Constructor/Destructor ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

RoboticArm::RoboticArm() 
    : pca9548a(Pca9548aParams{I2C_SCL_PIN, I2C_SDA_PIN, PCA9548A_SLAVE_ADDR, I2C_FREQ, I2C_NUM_0})
{
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

// AS5600_CONFigures FreeRTOS queues/semaphores/event groups
bool RoboticArm::initRTOSComms(){

    // Queues
    controlCmd = xQueueCreate(QUEUE_SIZE_USR_CMD, sizeof(UserCommand));
    kinematicsCmd = xQueueCreate(QUEUE_SIZE_USR_CMD, sizeof(UserCommand));

    // Creates queues for each motor's desired angle
    for (int i = 0; i < 6; i++){
        desiredAngleQueue[i] = xQueueCreate(QUEUE_SIZE_MOTOR, sizeof(float));
        paramsQueue[i]       = xQueueCreate(QUEUE_SIZE_MOTOR, sizeof(MotorParams));
    }

    // Task Notification


    // Event Groups
    motorEnable = xEventGroupCreate();
    motorIdle = xEventGroupCreate();
    motorReady = xEventGroupCreate();

    return this->errorCheckComms();

}

// Checks that all FreeRTOS Comms are initialized
bool RoboticArm::errorCheckComms(){
    bool error = false;

    // Queues
    if(controlCmd == NULL){
        ESP_LOGE(roboticArmInitTag, "Failed to create queue: controlCmd");
        error = true;
    }
    if(kinematicsCmd == NULL){
        ESP_LOGE(roboticArmInitTag, "Failed to create queue: kinematicsCmd");
        error = true;
    }

    for (int i = 0; i < 6; i++){
        if(desiredAngleQueue[i] == NULL){
            ESP_LOGE(roboticArmInitTag, "Failed to create queue: desiredAngleQueue[%d]", i);
            error = true;
        }
        if(desiredAngleQueue[i] == NULL){
            ESP_LOGE(roboticArmInitTag, "Failed to create queue: desiredAngleQueue[%d]", i);
            error = true;
        }
    }

    // Task Notifications


    // Event Groups
    if(motorEnable == NULL){
        ESP_LOGE(roboticArmInitTag, "Failed to create event group: motorEnable");
        error = true;
    }
    if(motorIdle == NULL){
        ESP_LOGE(roboticArmInitTag, "Failed to create event group: motorIdle");
        error = true;
    }

    return error;

}

// Initializes communcations task
bool RoboticArm::initCommunications(){
    //xTaskCreate(communicationTask, TASK_NAME_COMMUNICATION, TASK_STACK_DEPTH_COMMUNICATION, NULL, TASK_PRIORITY_COMMUNICATION, NULL);
    return false;
}

// Initializes central control task
bool RoboticArm::initControl(){
    xTaskCreate(controlTask, TASK_NAME_CONTROL, TASK_STACK_DEPTH_CONTROL, NULL, TASK_PRIORITY_CONTROL, NULL);
    return false;
}

// Initializes kinematics calculations task
bool RoboticArm::initKinematics(){
    //xTaskCreate(kinematicsTask, TASK_NAME_KINEMATICS, TASK_STACK_DEPTH_KINEMATICS, NULL, TASK_PRIORITY_KINEMATICS, NULL);
    return false;
}

// Initializes all motor tasks
bool RoboticArm::initAllMotors(){

    ESP_LOGI(motorInitTag, "Initializing all motors...");

    bool error = false;

    // Motor 1 Init
    if(this->initMotor1()){
        error = true;
    }

    // Motor 2 Init
    if(this->initMotor2()){
        error = true;
    }

    // Motor 3 Init
    if(this->initMotor3()){
        error = true;
    }

    // Motor 4 Init
    if(this->initMotor4()){
        error = true;
    }
    
    // Motor 5 Init
    if(this->initMotor5()){
        error = true;
    }

    // Motor 6 Init
    if(this->initMotor6()){
        error = true;
    }

    // Initialization Successful
    return error;

}

// Initializes motor task 1
bool RoboticArm::initMotor1(){

    // Motor 1
    if(this->pca9548a.pingDev(J1S_PORT, AS5600_ADDRESS)){
        // Device Found
        AS5600* j1sAS5600 = new AS5600(&this->pca9548a, J1S_PORT, AS5600_CONF);
        StepperMotor* j1sMotor = new StepperMotor(J1S_PIN_STEP, J1S_PIN_DIR);
        MotorParams* j1sParams = new MotorParams{j1sMotor, j1sAS5600, STEPPER_SPEED, -1, J1S_BIT_MASK, desiredAngleQueue[0], desiredAngleQueue[0]};
        xTaskCreate(motorTask, J1S_TASK_NAME, TASK_STACK_DEPTH_MOTOR, j1sParams, TASK_PRIORITY_MOTOR, NULL);
        ESP_LOGI(motorInitTag, "Motor 1 successfully initialized");
        return false;
    }
    else{
        // Device not found
        ESP_LOGE(motorInitTag, "Error initializing Motor 1");
        return true;
    }

}

// Initializes motor task 2
bool RoboticArm::initMotor2(){

    // Motor 2
    if(this->pca9548a.pingDev(J2S_PORT, AS5600_ADDRESS)){
        AS5600* j2sAS5600 = new AS5600(&this->pca9548a, J2S_PORT, AS5600_CONF);
        StepperMotor* j2sMotor = new StepperMotor(J2S_PIN_STEP, J2S_PIN_DIR);
        MotorParams* j2sParams = new MotorParams{j2sMotor, j2sAS5600, STEPPER_SPEED, -1, J2S_BIT_MASK, desiredAngleQueue[1], desiredAngleQueue[1]};
        xTaskCreate(motorTask, J2S_TASK_NAME, TASK_STACK_DEPTH_MOTOR, j2sParams, TASK_PRIORITY_MOTOR, NULL);
        ESP_LOGI(motorInitTag, "Motor 2 successfully initialized");
        return false;
    }
    else{
        ESP_LOGE(motorInitTag, "Error initializing Motor 2");
        return true;
    }

}

// Initializes motor task 3
bool RoboticArm::initMotor3(){

    // Motor 3
    if(this->pca9548a.pingDev(J3S_PORT, AS5600_ADDRESS)){
        AS5600* j3sAS5600 = new AS5600(&this->pca9548a, J3S_PORT, AS5600_CONF);
        StepperMotor* j3sMotor = new StepperMotor(J3S_PIN_STEP, J3S_PIN_DIR);
        MotorParams* j3sParams = new MotorParams{j3sMotor, j3sAS5600, STEPPER_SPEED, -1, J3S_BIT_MASK, desiredAngleQueue[2], desiredAngleQueue[2]};
        xTaskCreate(motorTask, J3S_TASK_NAME, TASK_STACK_DEPTH_MOTOR, j3sParams, TASK_PRIORITY_MOTOR, NULL);
        ESP_LOGI(motorInitTag, "Motor 3 successfully initialized");
        return false;
    }
    else{
        ESP_LOGE(motorInitTag, "Error initializing Motor 3");
        return true;
    }

}

// Initializes motor task 4
bool RoboticArm::initMotor4(){

    // Motor 4
    if(this->pca9548a.pingDev(J4S_PORT, AS5600_ADDRESS)){
        AS5600* j4sAS5600 = new AS5600(&this->pca9548a, J4S_PORT, AS5600_CONF);
        StepperMotor* j4sMotor = new StepperMotor(J4S_PIN_STEP, J4S_PIN_DIR);
        MotorParams* j4sParams = new MotorParams{j4sMotor, j4sAS5600, STEPPER_SPEED, -1, J4S_BIT_MASK, desiredAngleQueue[3], desiredAngleQueue[3]};
        xTaskCreate(motorTask, J4S_TASK_NAME, TASK_STACK_DEPTH_MOTOR, j4sParams, TASK_PRIORITY_MOTOR, NULL);
        ESP_LOGI(motorInitTag, "Motor 4 successfully initialized");
        return false;
    }
    else{
        ESP_LOGE(motorInitTag, "Error initializing Motor 4");
        return true;
    }

}

// Initializes motor task 5
bool RoboticArm::initMotor5(){

    // Motor 5
    if(this->pca9548a.pingDev(J5S_PORT, AS5600_ADDRESS)){
        AS5600* j5sAS5600 = new AS5600(&this->pca9548a, J5S_PORT, AS5600_CONF);
        StepperMotor* j5sMotor = new StepperMotor(J5S_PIN_STEP, J5S_PIN_DIR);
        MotorParams* j5sParams = new MotorParams{j5sMotor, j5sAS5600, STEPPER_SPEED, -1, J5S_BIT_MASK, desiredAngleQueue[4], desiredAngleQueue[4]};
        xTaskCreate(motorTask, J5S_TASK_NAME, TASK_STACK_DEPTH_MOTOR, j5sParams, TASK_PRIORITY_MOTOR, NULL);
        ESP_LOGI(motorInitTag, "Motor 5 successfully initialized");
        return false;
    }
    else{
        ESP_LOGE(motorInitTag, "Error initializing Motor 5");
        return true;
    }

}

// Initializes motor task 6
bool RoboticArm::initMotor6(){

    // Motor 6
    if(this->pca9548a.pingDev(J6S_PORT, AS5600_ADDRESS)){
        AS5600* j6sAS5600 = new AS5600(&this->pca9548a, J6S_PORT, AS5600_CONF);
        StepperMotor* j6sMotor = new StepperMotor(J6S_PIN_STEP, J6S_PIN_DIR);
        MotorParams* j6sParams = new MotorParams{j6sMotor, j6sAS5600, STEPPER_SPEED, -1, J6S_BIT_MASK, desiredAngleQueue[5], desiredAngleQueue[6]};
        xTaskCreate(motorTask, J6S_TASK_NAME, TASK_STACK_DEPTH_MOTOR, j6sParams, TASK_PRIORITY_MOTOR, NULL);
        ESP_LOGI(motorInitTag, "Motor 6 successfully initialized");
        return false;
    }
    else{
        ESP_LOGE(motorInitTag, "Error initializing Motor 6");
        return true;
    }

}

// Sends a UserCommand struct to the central control task
void RoboticArm::sendUserCommand(UserCommand* cmd){
    xQueueSend(controlCmd, cmd, 0);
}

// *********************************** PUBLIC **************************************

void RoboticArm::setEnd(float x, float y, float z, float pitch, float yaw, float roll){
    
    UserCommand cmd = {
        0,
        "setEnd",
        {x, y, z, pitch, yaw, roll}
    };
    
    this -> sendUserCommand(&cmd);

}

void RoboticArm::setEndSpeed(float speed){

    UserCommand cmd = {
        1,
        "setEndSpeed",
        {speed}
    };
    
    this -> sendUserCommand(&cmd);

}

void RoboticArm::setMotorAngle(int motor, float angle){

    UserCommand cmd = {
        10,
        "setMotorAngle",
        {(float)motor, angle}
    };
    
    this -> sendUserCommand(&cmd);

}

void RoboticArm::setMotorSpeed(int motor, float speed){

    UserCommand cmd = {
        11,
        "setMotorSpeed",
        {(float)motor, speed}
    };
    
    this -> sendUserCommand(&cmd);

}
