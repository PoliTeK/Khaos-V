#pragma once
#include <daisy_seed.h>
#include <dev/oled_ssd130x.h>

namespace khaos::display {
    using DisplayDriver = daisy::OledDisplay<daisy::SSD130x4WireSpi128x64Driver>;

    // SPI pin configuration
    constexpr daisy::Pin SCLK_PIN = daisy::seed::D8;
    constexpr daisy::Pin MOSI_PIN = daisy::seed::D10;
    constexpr daisy::Pin NSS_PIN  = daisy::seed::D7;

    // Other pins required by the display
    constexpr daisy::Pin DC_PIN = daisy::seed::D28;
    constexpr daisy::Pin RESET_PIN = daisy::seed::D2;

    daisy::UiCanvasDescriptor init(DisplayDriver &display);
}
