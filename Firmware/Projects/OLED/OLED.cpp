#include <stdio.h>
#include <string.h>
#include "daisy_seed.h"
#include "dev/oled_ssd130x.h"

using namespace daisy;

/** Typedef the OledDisplay to make syntax cleaner below 
 *  This is a 4Wire SPI Transport controlling an 128x64 sized SSDD1306
 * 
 *  There are several other premade test 
*/
using MyOledDisplay = OledDisplay<SSD130x4WireSpi128x64Driver>;

DaisySeed     hw;
MyOledDisplay display;

int main(void)
{
    hw.Configure();
    hw.Init();

    /** Configure the Display */
    MyOledDisplay::Config display_cfg;
    /** Set up the SPI Pins for SPI1 */
    display_cfg.driver_config.transport_config.spi_config.pin_config.sclk = seed::D8;
    display_cfg.driver_config.transport_config.spi_config.pin_config.mosi = seed::D10;
    display_cfg.driver_config.transport_config.spi_config.pin_config.nss = seed::D7;
    /** Command and Reset Pins */
    display_cfg.driver_config.transport_config.pin_config.dc    = seed::D28;
    display_cfg.driver_config.transport_config.pin_config.reset = seed::D2;
    /** And Initialize */
    display.Init(display_cfg);

    display.Update();
    System::Delay(3000);     // lo schermo dovrebbe restare tutto acceso 3 secondi

    /** Infinite Loop */
    while(1)
    {
        System::Delay(1000);

        /** Get the number of seconds since the program started */
        int seconds = System::GetNow() / 1000.0;

        /** Clear the display */
        display.Fill(false);
        /** Set the cursor away from the edge of the screen */
        display.SetCursor(4, 16);
        /** Write the following, starting at the cursor */
        display.WriteString("Time since startup:", Font_6x8, true);
        /** Update the cursor to the next space below the string */
        display.SetCursor(4, 32);

        /** Create a string representation of the */
        FixedCapStr<16> str("");
        str.AppendInt(seconds);
        /** Write the value to the display */
        display.WriteString(str, Font_11x18, true);

        /** And finally push the updated screen contents to the hardware */
        display.Update();
    }
}
