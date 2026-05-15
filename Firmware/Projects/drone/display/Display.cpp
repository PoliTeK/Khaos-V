#include "Display.hpp"

void SSD130X::InitDisplay(){
    SSD130xI2CTransport::Config display_transport_config;
    
    display_transport_config.i2c_config.pin_config.scl = seed::D11;
    display_transport_config.i2c_config.pin_config.sda = seed::D12;
    display_transport_config.i2c_config.speed = I2CHandle::Config::Speed::I2C_400KHZ;
    
    // Configure display
    daisy::OledDisplay<daisy::SSD130xI2c128x64Driver>::Config display_config;
    display_config.driver_config.transport_config = display_transport_config;
    
    display.Init(display_config);


    // model border set
    model_borders_t chua_b, sprott_b, rossler_b, halvorsen_b, lorentz_b;

    // This model for now has an exponential growth: it needs to be fixed
    chua_b.xr = 5000.0;
    chua_b.xl = -5000.0;
    chua_b.yr = 5000.0;
    chua_b.yl = -5000.0;
    chua_b.zr = 10.0;
    chua_b.zl = -10.0;

    sprott_b.xr = 2.12;
    sprott_b.xl = -0.99;
    sprott_b.yr = 1.31;
    sprott_b.yl = -2.006;
    sprott_b.zr = 1.915;
    sprott_b.zl = -1.895;

    rossler_b.xr = 11.431;
    rossler_b.xl = -9.104;
    rossler_b.yr = 7.839;
    rossler_b.yl = -10.789;
    rossler_b.zr = 22.838;
    rossler_b.zl = 0.013;

    halvorsen_b.xr = 6.358;
    halvorsen_b.xl = -12.239;
    halvorsen_b.yr = 6.331;
    halvorsen_b.yl = -12.347;
    halvorsen_b.zr = 6.334;
    halvorsen_b.zl = -12.186;

    // This model for now converges to a point: it needs to be fixed
    lorentz_b.xr = 10.0;
    lorentz_b.xl = -10.0;
    lorentz_b.yr = 10.0;
    lorentz_b.yl = -10.0;
    lorentz_b.zr = 10.0;
    lorentz_b.zl = -10.0;

    borders[static_cast<int>(Displayed_Models::CHUA)] = chua_b;
    borders[static_cast<int>(Displayed_Models::SPROTT)] = sprott_b;
    borders[static_cast<int>(Displayed_Models::ROSSLER)] = rossler_b;
    borders[static_cast<int>(Displayed_Models::HALVORSEN)] = halvorsen_b;
    borders[static_cast<int>(Displayed_Models::LORENTZ)] = lorentz_b;

    ClearDisplay();
}

void SSD130X::setCurrentModel(Displayed_Models md){
    model_number = md;
}

void SSD130X::ClearAll(){
    display.Fill(false);
}

void SSD130X::ClearDisplay(){
    display.Fill(false);
    display.DrawLine(63, 0, 63, 63, true);
    display.DrawLine(127, 0, 127, 63, true);
    display.DrawLine(63, 0, 127, 0, true);
    display.DrawLine(63, 63, 127, 63, true);

    for(int i=0; i<N_MODELS; i++){
        WriteText(i, models_names[i]);
    }

    SelectText(static_cast<int>(model_number));
}

void SSD130X::DrawPoint(math::vec3f state){
    uint8_t x, y;
    model_borders_t current_border = borders[static_cast<int>(model_number)];

    // from floating point value to a value between 0 and 63, representing the coordinates of the screen
    x = (state.x() - current_border.xl) / (current_border.xr - current_border.xl) * 63;
    y = (state.y() - current_border.yl) / (current_border.yr - current_border.yl) * 63;

    display.DrawPixel(63+x, y, true);
}

void SSD130X::UpdateDisplay(){
    display.Update();
}


// private methods

void SSD130X::WriteText(int pos, const char* text){
    display.SetCursor(2, 1+(pos*10));
    display.WriteString(text, Font_6x8, true);
}

void SSD130X::SelectText(int pos){
    display.DrawRect(1, pos*10, 60, pos*10+9, true, false);
}