// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Libraries ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include "config.h"
#include "roboticArm.h"

// Component Definitions
#include "as5600.h"
// #include "motorModule.h" **Defined in header**
// #include "pca9548a.h" **Defined in header**

// ESP/FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Tags
static const char *roboticArmInitTag = "Robotic Arm Initializer";
static const char *motorInitTag = "Motor Initializer";

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
    rtosResources.controlCmd = xQueueCreate(QUEUE_SIZE_USR_CMD, sizeof(UserCommand));
    rtosResources.kinematicsCmd = xQueueCreate(QUEUE_SIZE_USR_CMD, sizeof(UserCommand));

    // Creates queues for each motor's desired angle
    for (int i = 0; i < 6; i++){
        rtosResources.motorTargetsQueue[i] = xQueueCreate(QUEUE_SIZE_MOTOR, sizeof(TargetParams));
    }

    // Event Groups
    rtosResources.motorEnabled = xEventGroupCreate();
    rtosResources.motorIdle = xEventGroupCreate();
    rtosResources.motorReady = xEventGroupCreate();

    return this->errorCheckComms();

    

}

// Checks that all FreeRTOS Comms are initialized
bool RoboticArm::errorCheckComms(){
    bool error = false;

    // Queues
    if(rtosResources.controlCmd == NULL){
        ESP_LOGE(roboticArmInitTag, "Failed to create queue: controlCmd");
        error = true;
    }
    if(rtosResources.kinematicsCmd == NULL){
        ESP_LOGE(roboticArmInitTag, "Failed to create queue: kinematicsCmd");
        error = true;
    }

    for (int i = 0; i < 6; i++){
        if(rtosResources.motorTargetsQueue[i] == NULL){
            ESP_LOGE(roboticArmInitTag, "Failed to create queue: motorTargetsQueue[%d]", i);
            error = true;
        }
    }

    // Event Groups
    if(rtosResources.motorEnabled == NULL){
        ESP_LOGE(roboticArmInitTag, "Failed to create event group: motorEnabled");
        error = true;
    }
    if(rtosResources.motorIdle == NULL){
        ESP_LOGE(roboticArmInitTag, "Failed to create event group: motorIdle");
        error = true;
    }

    return error;

}

// Initializes communcations task
bool RoboticArm::initCommunications(){
    communicationTask.init(&rtosResources);
    communicationTask.start();
    return false;
}

// Initializes central control task
bool RoboticArm::initControl(){
    controlTask.init(&rtosResources);
    controlTask.start();
    return false;
}

// Initializes kinematics calculations task
bool RoboticArm::initKinematics(){
    kinematicsTask.init(&rtosResources, motors);
    kinematicsTask.start();
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
        motorTask[0].init(&rtosResources, J1S_TASK_NAME, motors[0]);
        motorTask[0].start();
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
        motorTask[1].init(&rtosResources, J2S_TASK_NAME, motors[1]);
        motorTask[1].start();
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
        motorTask[2].init(&rtosResources, J3S_TASK_NAME, motors[2]);
        motorTask[2].start();
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
        motorTask[3].init(&rtosResources, J4S_TASK_NAME, motors[3]);
        motorTask[3].start();
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
        motorTask[4].init(&rtosResources, J5S_TASK_NAME, motors[4]);
        motorTask[4].start();
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
        motorTask[5].init(&rtosResources, J6S_TASK_NAME, motors[5]);
        motorTask[5].start();
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
    motorMonitorTask.init(&rtosResources, motors);
    motorMonitorTask.start();
    return false;
}

// Sends a UserCommand struct to the central control task
void RoboticArm::sendUserCommand(UserCommand* cmd){
    xQueueSend(rtosResources.controlCmd, cmd, portMAX_DELAY);
    xQueueSend(rtosResources.kinematicsCmd, cmd, portMAX_DELAY);
}

// *********************************** PUBLIC **************************************

void RoboticArm::setEnd(float x, float y, float z, float pitch, float yaw, float roll){

    // DELETION CURRENTLY UNHANDLED
    Pose* posePtr = new Pose({
        {x, y, z},
        {pitch, yaw, roll}
    });
    
    UserCommand cmd = {
        "setEnd",
        static_cast<void*>(posePtr)
    };
    
    this -> sendUserCommand(&cmd);

}

void RoboticArm::setEndSpeed(float speed){

    // DELETION CURRENTLY UNHANDLED
    float* speedPtr = new float(speed);
    
    UserCommand cmd = {
        "setEndSpeed",
        static_cast<void*>(speedPtr)
    };
    
    this -> sendUserCommand(&cmd);

}

void RoboticArm::setAngles(float angle1, float angle2, float angle3, float angle4, float angle5, float angle6){

    float* angleArrayPtr = new float[6]{angle1, angle2, angle3, angle4, angle5, angle6};

    UserCommand cmd = {
        "setAngles",
        static_cast<void*>(angleArrayPtr)
    };
    
    this -> sendUserCommand(&cmd);

}

// Puts the robotic arm to sleep for a time in ms
void RoboticArm::sleep(int ms){

    int* msPtr = new int(ms);

    UserCommand cmd = {
        "sleep",
        static_cast<void*>(msPtr)
    };

    this -> sendUserCommand(&cmd);
}
