#include "display.hpp"

using namespace daisy;
using namespace khaos::display;

void clear_cd(const UiCanvasDescriptor& canvas) {
    auto display = ((DisplayDriver*) canvas.handle_);
    display->Fill(false);
}

void flush_cd(const UiCanvasDescriptor& canvas) {
    auto display = ((DisplayDriver*) canvas.handle_);
    display->Update();
}

UiCanvasDescriptor khaos::display::init(DisplayDriver& display) {
    UiCanvasDescriptor canvas{0, &display, 30, 0, false, clear_cd, flush_cd};

    DisplayDriver::Config display_cfg;

    // Set up the SPI Pins for SPI1
    display_cfg.driver_config.transport_config.spi_config.pin_config.sclk = SCLK_PIN;
    display_cfg.driver_config.transport_config.spi_config.pin_config.mosi = MOSI_PIN;
    display_cfg.driver_config.transport_config.spi_config.pin_config.nss = NSS_PIN;

    // Command and Reset Pins
    display_cfg.driver_config.transport_config.pin_config.dc    = DC_PIN;
    display_cfg.driver_config.transport_config.pin_config.reset = RESET_PIN;

    display.Init(display_cfg);    

    return canvas;
}

