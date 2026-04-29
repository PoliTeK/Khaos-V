#include "digipot.hpp"

namespace digipot {
    using namespace daisy;

    I2CHandle::Result init(I2CHandle &i2c) {
        I2CHandle::Config i2c_config;

        // choose which Daisy peripheral to use
        i2c_config.periph = I2C_PERIPHERAL;
        i2c_config.speed = I2CHandle::Config::Speed::I2C_100KHZ;
        i2c_config.mode = I2CHandle::Config::Mode::I2C_MASTER;

        // configure the I2C pins
        i2c_config.pin_config.scl = I2C_SCL;
        i2c_config.pin_config.sda = I2C_SDA;

        return i2c.Init(i2c_config);
    }

    uint8_t get_TCON_address(Wiper wiper) {
        switch (wiper)
        {
        case Wiper::Wiper0:
        case Wiper::Wiper1:
            return 0x4; // TCON0
        
        case Wiper::Wiper2:
        case Wiper::Wiper3:
            return 0xA; // TCON1

        default:
            return 0;
        }
    }

    I2CHandle::Result set_value(I2CHandle &i2c, Wiper wiper, uint16_t value) {
        uint8_t wiper_addr = static_cast<uint8_t>(wiper);

        uint8_t data[2];
        // Command: Write data. Four MSBs address the wiper.
        // Then two command bits (00 = write). Two LSBs are data bits D9:8. D9 is unused.
        data[0] = (wiper_addr << 4) | ((value >> 8) & 1); // A3:A0 0 0 D9:D8
        data[1] = value & 0xFF; // D7:0

        // Timeout in milliseconds
        return i2c.TransmitBlocking(I2C_ADDRESS, data, sizeof(data)/sizeof(*data), I2C_TIMEOUT_MS);
    }
}
