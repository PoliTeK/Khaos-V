#include <daisy_seed.h>
#include "inputs.hpp"

using namespace khaos::io;
using namespace daisy;


InputHandles::InputHandles(daisy::UiEventQueue& queue)
    : event_queue(queue) {}

void InputHandles::init_peripherals() {
    std::array<AdcChannelConfig, AdcChannels::ADC_NUM_CHANNELS> adc_config;

    // Setup Control Voltages
    adc_config[AdcChannels::ADC_CV0].InitSingle(seed::A0);
    adc_config[AdcChannels::ADC_CV1].InitSingle(seed::A1);
    adc.Init(adc_config.data(), adc_config.size());
    adc.Start();

    encoders[0].Init(seed::D17, seed::D18, seed::D24); // input 1
    encoders[1].Init(seed::D19, seed::D20, seed::D25); // input 2

    // Model selection: analog1, analog2 or digital
    encoders[2].Init(seed::D2, seed::D3, seed::D26);

    // Digital model selection
    encoders[3].Init(seed::D13, seed::D14, seed::D27);
}

RawInputData InputHandles::refresh(const RawInputData &prev_data) {
    auto data = prev_data;

    /* Encoders */
    for (size_t i = 0; i < encoders.size(); i++) {
        encoders[i].Debounce();

        // if 16 bits are not enough, there are problems worse than overflow...
        auto increments = static_cast<int16_t>(encoders[i].Increment());

        if (increments != 0) {
            // Update cumulative increments
            data.encoder_increments[i] += increments;
            event_queue.AddEncoderTurned(ENCODER_IDS[i], increments, ROTARY_ENCODER_RANGE_TICKS);
        }

        // Gather switch data
        // Count new presses on falling edges (arbitrary), but update state
        // on both rising and falling edge
        if (encoders[i].RisingEdge()) {
            event_queue.AddEncoderActivityChanged(ENCODER_IDS[i], true);
        }
        else if (encoders[i].FallingEdge()) {
            data.switches[i] += encoders[i].FallingEdge() ? 1 : 0;
            event_queue.AddEncoderActivityChanged(ENCODER_IDS[i], false);
        }
    }

    /* Control Voltages */
    data.cvs[0] = adc.Get(ADC_CV0);
    data.cvs[1] = adc.Get(ADC_CV1);

    return data;
}


