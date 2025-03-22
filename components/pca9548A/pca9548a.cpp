// ~~ Libraries ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "pca9548a.h"
#include "esp_log.h"


static const char *pca9584Tag = "PCA9548A Driver";


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

    // Creates Mutex for all i2c communications
    i2cMutex = xSemaphoreCreateMutex();

}

// ~~ Port Switch ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Sets the port the PCA9548A is set to.
// THIS IS NOT EXCLUSIVE TO PREVENT REDUNDANT MUTEX CALLS: BLOCK THE I2C BUS BEFORE USING
void PCA9548A::setPort(uint8_t port){

    esp_err_t err = i2c_master_write_to_device(params.i2cPort, params.slaveAddr, &port, 1, 50);

    // Check for errors
    if (err != ESP_OK) {
        ESP_LOGE(pca9584Tag, "I2C Port Error: %s", esp_err_to_name(err));
    }
}


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// *********************************** PUBLIC **************************************

// ~~ Port Switch ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Sets the port the PCA9548A is set to. This blocks I2C bus
void PCA9548A::setPortSafe(uint8_t port){

    // Blocks I2C Bus
    xSemaphoreTake(i2cMutex, portMAX_DELAY);

    this -> setPort(port);

    // Releases I2C Bus
    xSemaphoreGive(i2cMutex);


}

// ~~ Read/Write ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

uint8_t PCA9548A::readByte(uint8_t port, uint8_t addr, uint8_t reg){
    
    esp_err_t err;

    // Read buffer
    uint8_t data;

    // Blocks I2C Bus
    xSemaphoreTake(i2cMutex, portMAX_DELAY);

    // Sets the port
    this->setPort(port);

    // Set slave device register
    err = i2c_master_write_to_device(params.i2cPort, addr, &reg, 1, 50);

    if (err != ESP_OK) {
        // Releases I2C Bus
        xSemaphoreGive(i2cMutex);

        ESP_LOGE(pca9584Tag, "I2C Read Setup Error: %s", esp_err_to_name(err));
        return 0xFF;  // Return a default value on error
    }

    // Read from slave device
    err = i2c_master_read_from_device(params.i2cPort, addr, &data, 1, 50);

    // Releases I2C Bus
    xSemaphoreGive(i2cMutex);

    // Check for errors
    if (err != ESP_OK) {
        ESP_LOGE(pca9584Tag, "I2C Read Error: %s", esp_err_to_name(err));
        return 0xFF;  // Return a default value on error
    }

    // Return the received byte
    return data;
    
}

// Writes data to a given slave over a given port on the PCA9548A
void PCA9548A::write(uint8_t port, uint8_t addr, uint8_t* data, int numDataBytes){

    // Blocks I2C Bus
    xSemaphoreTake(i2cMutex, portMAX_DELAY);

    // Sets the port
    this->setPort(port);
    
    // Set slave device register
    esp_err_t err = i2c_master_write_to_device(params.i2cPort, addr, data, numDataBytes, 50);

    // Releases I2C Bus
    xSemaphoreGive(i2cMutex);
    
    if (err != ESP_OK) {
        ESP_LOGE(pca9584Tag, "I2C Write Error: %s", esp_err_to_name(err));
    }
}

// Pings slave device, returns true if ping is successful
bool PCA9548A::pingDev(uint8_t port, uint8_t addr){

    // Custom I2C command that sends the slave address, then checks for an ACK
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);

    // Blocks I2C Bus
    xSemaphoreTake(i2cMutex, portMAX_DELAY);

    // Sets the port
    this->setPort(port);

    // Runs Custom I2C Command
    esp_err_t err = i2c_master_cmd_begin(params.i2cPort, cmd, 50);

    // Releases I2C Bus
    xSemaphoreGive(i2cMutex);

    // Deletes Command
    i2c_cmd_link_delete(cmd);
    
    if (err == ESP_OK) {
        ESP_LOGI(pca9584Tag, "Device at 0x%02X found on port 0x%02X", addr, port);
        return true;
    } else if (err == ESP_ERR_TIMEOUT) {
        ESP_LOGE(pca9584Tag, "I2C Timeout while probing device at 0x%02X on port %02X", addr, port);
        return false;
    } else if (err == ESP_FAIL) {
        ESP_LOGE(pca9584Tag, "No device found at 0x%02X on port 0x%02X", addr, port);
        return false;
    } else {
        ESP_LOGE(pca9584Tag, "Error probing device at 0x%02X on port 0x%02X: %s", addr, port, esp_err_to_name(err));
        return false;
    }

}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
