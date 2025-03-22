#ifndef PCA9548A_H
#define PCA9548A_H

#include <vector>
#include "driver/i2c.h"
#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Port Definitions
#define PCA9548A_PORT_0 0b00000001
#define PCA9548A_PORT_1 0b00000010
#define PCA9548A_PORT_2 0b00000100
#define PCA9548A_PORT_3 0b00001000
#define PCA9548A_PORT_4 0b00010000
#define PCA9548A_PORT_5 0b00100000
#define PCA9548A_PORT_6 0b01000000
#define PCA9548A_PORT_7 0b10000000

// Parameters
struct Pca9548aParams{

    // Pin Definitions
    gpio_num_t i2cMasterScl;
    gpio_num_t i2cMasterSda;

    // I2C Characteristics
    uint8_t slaveAddr;

    uint32_t i2cFreq;
    i2c_port_t i2cPort;    // ESP32 i2c port number

};


class PCA9548A{
    private:

        // Parameters
        Pca9548aParams params; // PCA params;
        SemaphoreHandle_t i2cMutex;

        // Initialization
        void i2cInit();
        void devInit();

        // Helper Functions
        void setPort(uint8_t port);


    public:

         // Constructor/Destructor
        PCA9548A(const Pca9548aParams& params);
        ~PCA9548A();

        // Port Switch
        void setPortSafe(uint8_t port);

        // Read/Write
        uint8_t readByte(uint8_t port, uint8_t addr, uint8_t reg);
        void write(uint8_t port, uint8_t addr, uint8_t* data, int numDataBytes);

        // Ping Function
        bool pingDev(uint8_t port, uint8_t addr); // Pings slave device, returns true if ping is successful

};

#endif