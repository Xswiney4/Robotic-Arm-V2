#include "driver/i2c.h"
#include <vector>

// Port Definitions
#define PCA9548A_PORT_0 0b00000001
#define PCA9548A_PORT_1 0b00000010
#define PCA9548A_PORT_2 0b00000100
#define PCA9548A_PORT_3 0b00001000
#define PCA9548A_PORT_4 0b00010000
#define PCA9548A_PORT_5 0b00100000
#define PCA9548A_PORT_6 0b01000000
#define PCA9548A_PORT_7 0b10000000



// I2C Configuration
#define I2C_MASTER_SCL_IO           22           // Set SCL pin
#define I2C_MASTER_SDA_IO           21           // Set SDA pin
#define I2C_MASTER_NUM              I2C_NUM_0     // I2C port number
#define I2C_MASTER_FREQ_HZ          400000       // I2C frequency
#define I2C_MASTER_TX_BUF_DISABLE   0            // No buffer
#define I2C_MASTER_RX_BUF_DISABLE   0            // No buffer


class PCA9548A{
    private:
        // Private Variables
        uint8_t i2cMasterScl;       // SCL Pin
        uint8_t i2cMasterSda;       // SDA Pin
        uint8_t addr;               // Configured Slave Address


    public:

         // Constructor/Destructor
        PCA9548A();
        ~PCA9548A();

        // Read/Write
        std::vector<uint8_t> read(uint8_t port, uint8_t addr, int numBytes);
        bool write(uint8_t port, uint8_t addr, int numBytes);

        // Other I2C Commands
        bool pingSlave(uint8_t port, uint8_t addr);


};