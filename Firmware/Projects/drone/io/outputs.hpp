#pragma once

#include <daisy_seed.h>
#include "digipot/digipot.hpp"

namespace khaos::io {
    /// @brief Daisy Seed peripheral used to communicate with the digipots.
    constexpr daisy::I2CHandle::Config::Peripheral I2C_PERIPHERAL
        = daisy::I2CHandle::Config::Peripheral::I2C_1;

    // IMPORTANT: The SCL and SDA pins must match the peripheral!
    constexpr daisy::Pin I2C_SCL = daisy::seed::D11;
    constexpr daisy::Pin I2C_SDA = daisy::seed::D12;

    /**
     *  @brief Device address for the I2C protocol.
     *
     * The device uses internal addresses for each wiper, which are sent
     * separately during communication and unrelated to this one.
     */
    constexpr uint8_t I2C_DIGIPOT_ADDRESS = 0b0101100;

    constexpr uint32_t I2C_TIMEOUT_MS = 10;   

    //

    daisy::I2CHandle::Result init_i2c(daisy::I2CHandle &i2c);
    daisy::I2CHandle::Result set_potentiometer(daisy::I2CHandle& i2c, digipot::Wiper wiper, uint16_t value);
}
