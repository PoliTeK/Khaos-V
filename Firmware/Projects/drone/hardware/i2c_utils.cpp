#include "i2c_utils.hpp"

void i2c_scan(daisy::DaisySeed &hw, daisy::I2CHandle &i2c) {
    for(uint8_t addr = 1; addr < 127; addr++)
    {
        // provo a fare una trasmissione vuota
        daisy::I2CHandle::Result res = i2c.TransmitBlocking(addr, nullptr, 0, 10);

        if(res == daisy::I2CHandle::Result::OK)
        {
            hw.PrintLine("Found I2C device at 0x%02X\r\n", addr);
        }
    }
}

daisy::I2CHandle::Result i2c_check_addr(daisy::I2CHandle &i2c_handle, uint8_t addr)
{
	// provo a fare una trasmissione vuota
	// nota: il timeout è in ms
	return i2c_handle.TransmitBlocking(addr, nullptr, 0, 10);
}
