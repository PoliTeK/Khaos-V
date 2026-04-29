#pragma once
#include "daisy_seed.h"

void i2c_scan(daisy::DaisySeed &hw, daisy::I2CHandle &i2c);

daisy::I2CHandle::Result i2c_check_addr(daisy::I2CHandle &i2c_handle, uint8_t addr);
