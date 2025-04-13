
#include "motorModule.h"
#include "config.h"

#include <chrono> // For time in ms
#include <thread> // For sleeping
#include <cstdint> // for uint16_t
#include <iostream>
#include <iomanip>
#include "esp_log.h"
#include <cmath>

#define TESTCOUNT 10

extern "C" void app_main(){

    Pca9548aParams pca9548aParams = {I2C_SCL_PIN, I2C_SDA_PIN, PCA9548A_SLAVE_ADDR, I2C_FREQ, I2C_NUM_0};
    PCA9548A pca9548a(pca9548aParams);

    MotorParams params1;
    params1.pca9548a = &pca9548a;
    params1.pca9548aPort = J2S_PORT;
    params1.as5600Config = AS5600_CONF;
    params1.bitMask = J2S_BIT_MASK;
    params1.dirPin = J2S_PIN_DIR;
    params1.stepPin = J2S_PIN_STEP;

    MotorParams params2;
    params2.pca9548a = &pca9548a;
    params2.pca9548aPort = J6S_PORT;
    params2.as5600Config = AS5600_CONF;
    params2.bitMask = J6S_BIT_MASK;
    params2.dirPin = J6S_PIN_DIR;
    params2.stepPin = J6S_PIN_STEP;

    MotorModule motor1(params1);
    MotorModule motor2(params2);

    // float startAngle = motor1.updateAngle();

    // ESP_LOGI("MOTOR TEST", "Start Angle: %.2f", startAngle);

    // motor1.setDir(COUNTERCLOCKWISE);
    // std::this_thread::sleep_for(std::chrono::microseconds(2));


    // for(int i = 0; i < TESTCOUNT; i++){
    //     motor1.step();
    //     std::this_thread::sleep_for(std::chrono::microseconds(1000));
    // }

    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // float newAngle = motor1.updateAngle();
    // float difference = newAngle - startAngle;
    // float error = difference - (1.8 * TESTCOUNT);

    // ESP_LOGI("MOTOR TEST", "Angle after steps %.2f", newAngle);
    // ESP_LOGI("MOTOR TEST", "Difference = %.2f", difference );
    // ESP_LOGI("MOTOR TEST", "Error = %.2f", error );





    motor2.setAngle(0.0f, STEPPER_SPEED);
    // motor1.setAngle(90.0f, STEPPER_SPEED);
    // motor1.setAngle(180.0f, STEPPER_SPEED);
    // motor1.setAngle(270.0f, STEPPER_SPEED);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    motor2.setAngle(360.0f, STEPPER_SPEED);
    // motor1.setAngle(270.0f, STEPPER_SPEED);
    // motor1.setAngle(180.0f, STEPPER_SPEED);
    // motor1.setAngle(90.0f, STEPPER_SPEED);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    motor2.setAngle(0.0f, STEPPER_SPEED);
    // motor1.setAngle(90.0f, STEPPER_SPEED);
    // motor1.setAngle(180.0f, STEPPER_SPEED);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    motor2.setAngle(270.0f, STEPPER_SPEED - 150.0f);
    
    while(true){
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }

}