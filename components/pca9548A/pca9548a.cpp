// ~~ Libraries ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "pca9548a.h"
#include <iostream>

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~ Constructor/Destructor ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Constructor: Creates and initializes object
PCA9548A::PCA9548A(const Pca9548aParams& params): params(params){
    
    i2cInit();

}

PCA9548A::~PCA9548A(){
    // Cleanup i2c Bus
    i2c_driver_delete(params.i2cPort);
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ********************************* PRIVATE **************************************

// ~~ Init ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void PCA9548A::i2cInit(){
    // I2C conf builder
    i2c_config_t conf;

    std::cout << "I2C Frequency: " << params.i2cFreq << std::endl;
    conf.mode =                 I2C_MODE_MASTER;        // Default
    conf.sda_io_num =           params.i2cMasterSda;    // SDA Pin
    conf.scl_io_num =           params.i2cMasterScl;    // SCL Pin
    conf.sda_pullup_en =        GPIO_PULLUP_ENABLE;     // Internal pull up
    conf.scl_pullup_en =        GPIO_PULLUP_ENABLE;     // Internal pull up
    conf.master.clk_speed =     params.i2cFreq;         // Clock speed
    conf.clk_flags =            I2C_SCLK_SRC_FLAG_FOR_NOMAL;  // Normal clock source
    

    // Attempts to setup i2c, returns error if failure
    ESP_ERROR_CHECK(i2c_param_config(params.i2cPort, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(params.i2cPort, I2C_MODE_MASTER, 0, 0, 0));

}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// *********************************** PUBLIC **************************************

// ~~ Port Switch ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void PCA9548A::setPort(uint8_t port){

    esp_err_t err = i2c_master_write_to_device(params.i2cPort, params.slaveAddr, &port, 1, 50);

    // Check for errors
    if (err != ESP_OK) {
        printf("I2C Port Error: %s\n", esp_err_to_name(err));
    }
}

// ~~ Read/Write ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

uint8_t PCA9548A::readByte(uint8_t port, uint8_t addr, uint8_t reg){
    
    // Sets the port
    this->setPort(port);

    esp_err_t err;

    // Read buffer
    uint8_t data;

    // Set slave device register
    err = i2c_master_write_to_device(params.i2cPort, addr, &reg, 1, 50);

    if (err != ESP_OK) {
        printf("I2C Read Setup Error: %s\n", esp_err_to_name(err));
        return 0xFF;  // Return a default value on error
    }

    // Read from slave device
    err = i2c_master_read_from_device(params.i2cPort, addr, &data, 1, 50);

    // Check for errors
    if (err != ESP_OK) {
        printf("I2C Read Error: %s\n", esp_err_to_name(err));
        return 0xFF;  // Return a default value on error
    }

    // Return the received byte
    return data;
    
}

// Writes data to a given slave over a given port on the PCA9548A
void PCA9548A::write(uint8_t port, uint8_t addr, uint8_t* data, int numDataBytes){

    /* Writing to the PCA9548A follows this format:
    Start
    Send PCA9548A address (write) [0x70]    - Byte 1
    Send control register                   - Byte 2
    Send slave device address (write)       - Byte 3
    Send register address (if applicable)   - Byte 4
    Send data                               - Data Bytes
    Stop
    */

    // Set slave device register
    esp_err_t err = i2c_master_write_to_device(params.i2cPort, addr, data, numDataBytes, 50);
    
    if (err != ESP_OK) {
        printf("I2C Write Error: %s\n", esp_err_to_name(err));
    }
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
