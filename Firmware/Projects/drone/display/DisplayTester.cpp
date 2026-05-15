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
int main(void)
{
    char coords[2][12];

    hw.Configure();
    hw.Init();

    display.InitDisplay();

    display.setCurrentModel(Displayed_Models::ROSSLER); // Change this variable for model selection in the list
    
    dcblock.Init(1);
    
    math::DiscretizedModel<math::Rossler> model({}, 0.01); // Change the object model here for model selection in the graph
    model.dt = 0.05f; // Speed of the model
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

        for (int i = 0; i < 20; i++) {
            state = model.step(state);

            display.DrawPoint(state);
        }

        sprintf(coords[0], FLT_FMT(3), FLT_VAR(3,state.x()));
        sprintf(coords[1], FLT_FMT(3), FLT_VAR(3,state.y()));

        display.UpdateDisplay();

        hw.DelayMs(1000/60);
    }
}
