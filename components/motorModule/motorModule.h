#ifndef MOTOR_GEARBOX_SYSTEM_H
#define MOTOR_GEARBOX_SYSTEM_H

#include "driver/gpio.h"
#include "as5600.h"
#include <optional>

// Directional Variables
#define CLOCKWISE           true
#define COUNTERCLOCKWISE    false

// Gear Ratios
#define GEAR_RATIO_1_TO_1       1.0f
#define GEAR_RATIO_3_TO_2       1.5f
#define GEAR_RATIO_2_TO_1       2.0f
#define GEAR_RATIO_10_TO_1      10.0f
#define GEAR_RATIO_15_TO_1      15.0f

// Microstepping
#define MICROSTEPPING_FULL          1.0f
#define MICROSTEPPING_HALF          2.0f
#define MICROSTEPPING_QUARTER       4.0f
#define MICROSTEPPING_EIGHTH        8.0f
#define MICROSTEPPING_SIXTEENTH     16.0f

struct MotorParams{

    // GPIO Pins
    gpio_num_t stepPin;
    gpio_num_t dirPin;

    // AS5600 Setup
    PCA9548A* pca9548a;
    uint8_t pca9548aPort;
    uint16_t as5600Config;

    // Motor Parameters
    float gearRatio = GEAR_RATIO_1_TO_1;
    float microstepping = MICROSTEPPING_FULL;
    float baseDegreesPerStep = 1.8f;    // Degrees the motor move's per step at full steps.

    // FreeRTOS Variables
    uint8_t bitMask = 0;
    
};

struct TargetParams{
    int numSteps;           // Total steps to traverse
    float targetAngle;      // Target angle to reach, used for validation
    TickType_t xFrequency;  // Hz
};

class MotorModule{
    private:

        // Private Variables
        MotorParams motorParams;
        TargetParams targetParams;
        std::optional<AS5600> as5600;
        portMUX_TYPE stepMux = portMUX_INITIALIZER_UNLOCKED;

        // Raw Angles
        float lastCurrentAngle;  // Last measured as5600 measurement
        float currentAngle;
        float startAngle;     // Angle system started on before moving
        bool currentDir;

        // Setup
        void setupGPIO();
        void setupAS5600();
        void calibrate();
        
        // Helper Methods
        int degToSteps(float degrees);
        float stepsToDeg(int steps);

        // Servo Controls through Setup
        void setupTargetSteps(int numSteps);
        void setupTargetAngle(float targetAngle);
        void setupTargetSpeed(float speed);
        void setupTargetFrequency(TickType_t xFrequency);
        void moveToTarget();
        
    public:
        // Constructor/Destructor
        MotorModule(MotorParams params);
        ~MotorModule();

        float degreesPerStep;   // Degrees the output moves for each step

        // Motor Identification
        uint8_t bitMask;
        int motorNum;

        // Motor Controls
        void step();
        void step(bool dir);
        void setDir(bool dir);
        void toggleDir();

        // Angle Measurements
        float updateAngle();
        
        // Validation
        bool validateCurrentStep();

        // Auto-Start Controls
        void setAngle(float angle, float speed);
        void setAngle(float angle, TickType_t xFrequency);
        void setAngle(int numSteps, float speed);
        void setAngle(int numSteps, TickType_t xFrequency);
        void setAngle(TargetParams params);

};

#endif