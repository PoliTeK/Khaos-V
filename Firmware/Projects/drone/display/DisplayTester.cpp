#include "daisy_seed.h"
#include "daisysp.h"
#include "Display.hpp"
#include "../math/models.hpp"

using namespace daisy;

DaisySeed hw;
SSD130X display;
daisysp::DcBlock dcblock;

constexpr size_t NUM_MODELS = 3;
math::ContinuousModel<math::vec3f> *models[NUM_MODELS];
math::vec3f states[NUM_MODELS];

daisy::TimerHandle timer;

// Call from prototype.cpp/main() if needed
int DisplayTester_main(void)
{
    uint8_t out[2];
    hw.Configure();
    hw.Init();

    display.InitDisplay();
    char coords[2][12];
    dcblock.Init(1);

    math::DiscretizedModel<math::Halvorsen> model({}, 0.01);
    model.dt = 0.0025f;
    math::vec3f state = math::vec3f{1.0, 1.0, -1.0};

    TimerHandle::Config config;
    config.dir = TimerHandle::Config::CounterDir::UP;
    config.enable_irq = true;
    config.periph = TimerHandle::Config::Peripheral::TIM_2;
    timer.Init(config);
    timer.SetPeriod(timer.GetFreq()/20);

    while(1)
    {
        display.ClearDisplay();
        display.WriteText(0,"Lorentz");
        display.WriteText(1,"Rossler");
        display.WriteText(2,"Halvorsen");
        display.SelectText(2);

        for (int i = 0; i < 20; i++) {
            state = model.step(state);

            out[0] = state.x() * 2 + 32;
            out[1] = state.y() * 2 + 32;

            display.DrawPoint(out[0], out[1]);
        }

        sprintf(coords[0], FLT_FMT(3), FLT_VAR(3,state.x()));
        sprintf(coords[1], FLT_FMT(3), FLT_VAR(3,state.y()));

        display.WriteText(4, coords[0]);
        display.WriteText(5, coords[1]);
        display.UpdateDisplay();

        hw.DelayMs(1000/20);
    }
}
