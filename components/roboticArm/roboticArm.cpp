// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Libraries ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include "config.h"
#include "roboticArm.h"

// Component Definitions
#include "as5600.h"
#include "stepper.h"
// #include "motorModule.h" **Defined in header**
// #include "pca9548a.h" **Defined in header**

// Tasks Definitions
#include "communication_task.h"
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
QueueHandle_t controlCmd;       // Queue for user commands      (UserCommand Struct)
QueueHandle_t kinematicsCmd; // Queue for kinematics task    (UserCommand Struct)

QueueHandle_t motorTargetsQueue[6]; // Queue for stepper motor angles

// Event Groups
EventGroupHandle_t motorEnabled; // Enables Motors
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

    // Motor Tasks
    if(this->initAllMotors()){
        ESP_LOGE(roboticArmInitTag, "Error initializing Motor Tasks");
        //return true;
    }
    else{
        ESP_LOGI(roboticArmInitTag, "Motor Tasks successfully initialized");
    }

    // Motor Monitor
    if(this->initMotorMonitor()){
        ESP_LOGE(roboticArmInitTag, "Error initializing Motor Monitor Task");
        return true;
    }
    else{
        ESP_LOGI(roboticArmInitTag, "Motor Monitor Task successfully initialized");
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
        motorTargetsQueue[i] = xQueueCreate(QUEUE_SIZE_MOTOR, sizeof(TargetParams));
    }

    // Event Groups
    motorEnabled = xEventGroupCreate();
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
        if(motorTargetsQueue[i] == NULL){
            ESP_LOGE(roboticArmInitTag, "Failed to create queue: motorTargetsQueue[%d]", i);
            error = true;
        }
    }

    // Event Groups
    if(motorEnabled == NULL){
        ESP_LOGE(roboticArmInitTag, "Failed to create event group: motorEnabled");
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
    xTaskCreate(kinematicsTask, TASK_NAME_KINEMATICS, TASK_STACK_DEPTH_KINEMATICS, motors, TASK_PRIORITY_KINEMATICS, NULL);
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

        // Parameter Creation
        MotorParams params;

        params.stepPin =            J1S_PIN_STEP;
        params.dirPin =             J1S_PIN_DIR;

        params.pca9548a =           &pca9548a;
        params.pca9548aPort =       J1S_PORT;
        params.as5600Config =       AS5600_CONF;

        params.gearRatio =          J1S_GEAR_RATIO;
        params.microstepping =      J1S_MICROSTEP;
        params.baseDegreesPerStep = J1S_DEG_P_STEP;

        params.bitMask =            J1S_BIT_MASK;

        // MotorModule Creation
        motors[0] = new MotorModule(params);

        // Task Creation
        xTaskCreate(motorTask, J1S_TASK_NAME, TASK_STACK_DEPTH_MOTOR, motors[0], TASK_PRIORITY_MOTOR, NULL);
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
        // Device Found

        // Parameter Creation
        MotorParams params;

        params.stepPin =            J2S_PIN_STEP;
        params.dirPin =             J2S_PIN_DIR;

        params.pca9548a =           &pca9548a;
        params.pca9548aPort =       J2S_PORT;
        params.as5600Config =       AS5600_CONF;

        params.gearRatio =          J2S_GEAR_RATIO;
        params.microstepping =      J2S_MICROSTEP;
        params.baseDegreesPerStep = J2S_DEG_P_STEP;

        params.bitMask =            J2S_BIT_MASK;

        // MotorModule Creation
        motors[1] = new MotorModule(params);

        // Task Creation
        xTaskCreate(motorTask, J2S_TASK_NAME, TASK_STACK_DEPTH_MOTOR, motors[1], TASK_PRIORITY_MOTOR, NULL);
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
        // Device Found

        // Parameter Creation
        MotorParams params;

        params.stepPin =            J3S_PIN_STEP;
        params.dirPin =             J3S_PIN_DIR;

        params.pca9548a =           &pca9548a;
        params.pca9548aPort =       J3S_PORT;
        params.as5600Config =       AS5600_CONF;

        params.gearRatio =          J3S_GEAR_RATIO;
        params.microstepping =      J3S_MICROSTEP;
        params.baseDegreesPerStep = J3S_DEG_P_STEP;

        params.bitMask =            J3S_BIT_MASK;

        // MotorModule Creation
        motors[2] = new MotorModule(params);

        // Task Creation
        xTaskCreate(motorTask, J3S_TASK_NAME, TASK_STACK_DEPTH_MOTOR, motors[2], TASK_PRIORITY_MOTOR, NULL);
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
        // Device Found

        // Parameter Creation
        MotorParams params;

        params.stepPin =            J4S_PIN_STEP;
        params.dirPin =             J4S_PIN_DIR;

        params.pca9548a =           &pca9548a;
        params.pca9548aPort =       J4S_PORT;
        params.as5600Config =       AS5600_CONF;

        params.gearRatio =          J4S_GEAR_RATIO;
        params.microstepping =      J4S_MICROSTEP;
        params.baseDegreesPerStep = J4S_DEG_P_STEP;

        params.bitMask =            J4S_BIT_MASK;

        // MotorModule Creation
        motors[3] = new MotorModule(params);

        // Task Creation
        xTaskCreate(motorTask, J4S_TASK_NAME, TASK_STACK_DEPTH_MOTOR, motors[3], TASK_PRIORITY_MOTOR, NULL);
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
        // Device Found

        // Parameter Creation
        MotorParams params;

        params.stepPin =            J5S_PIN_STEP;
        params.dirPin =             J5S_PIN_DIR;

        params.pca9548a =           &pca9548a;
        params.pca9548aPort =       J5S_PORT;
        params.as5600Config =       AS5600_CONF;

        params.gearRatio =          J5S_GEAR_RATIO;
        params.microstepping =      J5S_MICROSTEP;
        params.baseDegreesPerStep = J5S_DEG_P_STEP;

        params.bitMask =            J5S_BIT_MASK;

        // MotorModule Creation
        motors[4] = new MotorModule(params);

        // Task Creation
        xTaskCreate(motorTask, J5S_TASK_NAME, TASK_STACK_DEPTH_MOTOR, motors[4], TASK_PRIORITY_MOTOR, NULL);
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
        // Device Found

        // Parameter Creation
        MotorParams params;

        params.stepPin =            J6S_PIN_STEP;
        params.dirPin =             J6S_PIN_DIR;

        params.pca9548a =           &pca9548a;
        params.pca9548aPort =       J6S_PORT;
        params.as5600Config =       AS5600_CONF;

        params.gearRatio =          J6S_GEAR_RATIO;
        params.microstepping =      J6S_MICROSTEP;
        params.baseDegreesPerStep = J6S_DEG_P_STEP;

        params.bitMask =            J6S_BIT_MASK;

        // MotorModule Creation
        motors[5] = new MotorModule(params);

        // Task Creation
        xTaskCreate(motorTask, J6S_TASK_NAME, TASK_STACK_DEPTH_MOTOR, motors[5], TASK_PRIORITY_MOTOR, NULL);
        ESP_LOGI(motorInitTag, "Motor 6 successfully initialized");
        
        return false;
    }
    else{
        ESP_LOGE(motorInitTag, "Error initializing Motor 6");
        return true;
    }

}

// Initializes the motor task monitor
bool RoboticArm::initMotorMonitor(){
    xTaskCreate(stepMonitorTask, TASK_NAME_STEP_MONITOR, TASK_STACK_DEPTH_STEP_MONITOR, motors, TASK_PRIORITY_STEP_MONITOR, NULL);
    return false;
}

// Sends a UserCommand struct to the central control task
void RoboticArm::sendUserCommand(UserCommand* cmd){
    xQueueSend(controlCmd, cmd, portMAX_DELAY);
    xQueueSend(kinematicsCmd, cmd, portMAX_DELAY);
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

void RoboticArm::setMotorAngles(float angle1, float angle2, float angle3, float angle4, float angle5, float angle6){

    UserCommand cmd = {
        10,
        "setMotorAngles",
        {angle1, angle2, angle3, angle4, angle5, angle6}
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

// Puts the robotic arm to sleep for a time in ms
void RoboticArm::sleep(int ms){
    UserCommand cmd = {
        20,
        "sleep",
        {(float)ms}
    };

    this -> sendUserCommand(&cmd);
}
