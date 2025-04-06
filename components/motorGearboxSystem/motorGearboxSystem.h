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

    // Bit Mask
    uint8_t bitMask;
    
};

struct TargetParams{
    float targetAngle = -1.0f;     // Degrees
    TickType_t xFrequency = -1; // Hz
};

class MotorGearboxSystem{
    private:

        // Private Variables
        MotorParams motorParams;
        TargetParams targetParams;
        std::optional<AS5600> as5600;

        // Angle Variables
        bool currentDir;
        float inputAngle;
        float maxAngle;

        // Setup
        void setupGPIO();
        void setupAS5600();
        void calibrate();
        
    public:
        // Constructor/Destructor
        MotorGearboxSystem(MotorParams params);
        ~MotorGearboxSystem();

        // Bit Mask
        uint8_t bitMask;

        // Motor Controls
        void step();
        void step(bool dir);
        void setDir(bool dir);

        // Angle Measurements
        float getRawAngle();
        float getAngle();

        // Servo Controls through Setup
        void setupTargetAngle(float angle);
        void setupTargetSpeed(float speed);
        void setupTargetFrequency(TickType_t xFrequency);
        void resetTarget();
        void start();

        // Auto-Start Controls
        void setTargetAngle(float angle, float speed);
        void setTargetAngle(float angle, TickType_t xFrequency);

};

#endif