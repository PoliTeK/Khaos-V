/Users/federicolessio/Desktop/poli/Khaos-V/Firmware/Projects/drone/display/Display.cpp#include "daisy_seed.h"
#include "daisysp.h"
#include "Display.hpp"
#include "../math/models.hpp"

using namespace daisy;

/* This code is used only to search for the values of the boundaries of the models empirically,
which then will be used on the display to fit the model inside the screen
*/

DaisySeed hw;
SSD130X display;
daisysp::DcBlock dcblock;

constexpr size_t NUM_MODELS = 3;
math::ContinuousModel<math::vec3f> *models[NUM_MODELS];
math::vec3f states[NUM_MODELS];

daisy::TimerHandle timer;

// Call from prototype.cpp/main() if needed
int main(void)
{
    float max[6] = {-10.0, // Right x bound
        10.0, // Left x bound
        -10.0, // Right y bound
        10.0, // Left y bound
        -10.0, // Right z bound
        10.0}; // Left z bound

    hw.Configure();
    hw.Init();

    display.InitDisplay();
    char coords[6][18] = {
        "Model x+: ",
        "Model x-: ",
        "Model y+: ",
        "Model y-: ",
        "Model z+: ",
        "Model z-: "
    };
    char temp[8];
    dcblock.Init(1);

    math::DiscretizedModel<math::Rossler> model({}, 0.01); // All models to test: Chua, Sprott, Rossler, Halvorsen, Lorentz
    model.dt = 0.01f;
    math::vec3f state = math::vec3f{-1.0, -1.0, 1.0};

    TimerHandle::Config config;
    config.dir = TimerHandle::Config::CounterDir::UP;
    config.enable_irq = true;
    config.periph = TimerHandle::Config::Peripheral::TIM_2;
    timer.Init(config);
    timer.SetPeriod(timer.GetFreq()/20);

    while(1)
    {
        display.ClearAll();
        sprintf(coords[0], "Model x+: ");
        sprintf(coords[1], "Model x-: ");
        sprintf(coords[2], "Model y+: ");
        sprintf(coords[3], "Model y-: ");
        sprintf(coords[4], "Model z+: ");
        sprintf(coords[5], "Model z-: ");

        for (int i = 0; i < 20; i++) {
            state = model.step(state);
            max[0] = (state.x() > max[0]) ? state.x() : max[0];
            max[1] = (state.x() < max[1]) ? state.x() : max[1];
            max[2] = (state.y() > max[2]) ? state.y() : max[2];
            max[3] = (state.y() < max[3]) ? state.y() : max[3];
            max[4] = (state.z() > max[4]) ? state.z() : max[4];
            max[5] = (state.z() < max[5]) ? state.z() : max[5];
        }

        sprintf(temp, FLT_FMT(3), FLT_VAR(3,max[0]));
        strcat(coords[0], temp);
        sprintf(temp, FLT_FMT(3), FLT_VAR(3,max[1]));
        strcat(coords[1], temp);
        sprintf(temp, FLT_FMT(3), FLT_VAR(3,max[2]));
        strcat(coords[2], temp);
        sprintf(temp, FLT_FMT(3), FLT_VAR(3,max[3]));
        strcat(coords[3], temp);
        sprintf(temp, FLT_FMT(3), FLT_VAR(3,max[4]));
        strcat(coords[4], temp);
        sprintf(temp, FLT_FMT(3), FLT_VAR(3,max[5]));
        strcat(coords[5], temp);

        display.WriteText(0, coords[0]);
        display.WriteText(1, coords[1]);
        display.WriteText(2, coords[2]);
        display.WriteText(3, coords[3]);
        display.WriteText(4, coords[4]);
        display.WriteText(5, coords[5]);
        display.UpdateDisplay();

        hw.DelayMs(1000/20);
    }
}
