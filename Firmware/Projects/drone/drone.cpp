/* Khaos-V */
/* PoliTeK 2026*/

// This file contains code specialized for the final revision of the project
// layout.
// To prototype new code, use prototype/prototype.cpp

#include <atomic>
#include <daisy_seed.h>

#include <array>
#include <limits>

#include "math/vecmath.hpp"
// Chaotic models
#include "math/chaos_osc.hpp"
#include "math/models.hpp"

#include "hardware/digipot.hpp"
#include "hardware/i2c_utils.hpp"

#include "per/adc.h"
#include "per/i2c.h"
#include "sync/TriBuf.hpp"

using namespace daisy;

/* --- Configuration ---------------------------------------------------------------------------- */
constexpr bool DEBUG = true;
constexpr const char *LOG_LABEL = "[Khaos-V]";
constexpr std::array<const char *, 2> LOG_RESULT{"Error", "Success"};

constexpr uint32_t INPUT_SAMPLE_RATE = 1;     // per second
constexpr uint32_t OUTPUT_SAMPLE_RATE = 100;  // per second
constexpr uint32_t DISPLAY_REFRESH_RATE = 30; // per second

/// Number of samples stored in the output buffer
constexpr size_t OUTPUT_BUFFER_SIZE = 128;

/// @brief Sets how many 'ticks' cover the full range.
/// Controls the resolution for encoder-controlled parameters.
constexpr uint16_t ROTARY_ENCODER_RESOLUTION = 32;
constexpr size_t PARAM_RESOLUTION = 1024;

/* --- Definitions ------------------------------------------------------------------------------ */

struct KhaosInputData;

/**
 * Contains all hardware handles related to inputs.
 * It is safely shared between interrupt callbacks because it does not contain
 * actual input data, as long as only one "process" updates it.
 */
struct KhaosInput {
    std::atomic_bool pending_refresh{false};

    AdcHandle adc;
    std::array<Encoder, 4> encoders;

    enum AdcChannels { ADC_CV0 = 0, ADC_CV1, ADC_NUM_CHANNELS };

    /// @brief Initialize ADC channels and GPIOs
    void init();
    void refresh(KhaosInputData &);
};

/** Contains all hardware handles related to outputs.
 *  It is safely shared between interrupt callbacks because it does not contain
 *  actual output data.
 */
struct KhaosOutput {
    DacHandle dac;
    std::array<GPIO, 3> leds;

    /// @brief Initialize DAC channels
    void init();
};

/**
 * Input data coming from external hardware.
 * This must be synchronized across interrupts and processed in order to be
 * used to control chaotic models and other outputs.
 */
struct KhaosInputData {
    std::array<uint16_t, 4> encoder_values;
    std::array<bool, 4> switches;
    std::array<uint16_t, 2> cvs;

    KhaosInputData();
};

// TODO: choose models
struct KhaosModelData {
    enum SelectedModel { ROSSLER = 0, NUM_MODELS } selected;
    math::Rossler rossler;
};

/// @brief Initializes timers
void init_timers();

void input_timer_callback(void *data);
void output_dma_callback(uint16_t **out, size_t size);
void display_refresh_callback(void *data);

// /// @brief Initialized chaotic models with default parameters
// void init_chaotic_models();

/* --- Global variables ------------------------------------------------------------------------- */

/// TIM3: 16-bit timer
TimerHandle input_timer;
/// TIM4: 16-bit timer
TimerHandle display_timer;

static KhaosInput input;
static KhaosOutput output;

static TriBuf<KhaosModelData> model_data;
static TriBuf<KhaosModelData>::Writer model_data_writer; // owner: main
static TriBuf<KhaosModelData>::Reader model_data_reader; // owner: output_dma_callback

static std::array<std::array<uint16_t, OUTPUT_BUFFER_SIZE>, 2> output_buf;

/* --- Main code -------------------------------------------------------------------------------- */

int main() {
    DaisySeed hw;

    // Needed to maintain a persistent state, even if the reader loses some updates
    KhaosInputData input_data;

    hw.Init();
    hw.StartLog(DEBUG);
    hw.PrintLine("%s Starting initialization...", LOG_LABEL);

    // no one has access to these yet
    // we must ensure that at most one "process" (i.e. interrupt callback)
    // has access to one of these at any time
    model_data_writer = model_data.get_writer();
    model_data_reader = model_data.get_reader();

    hw.PrintLine("%s Acquired TriBuf handles", LOG_LABEL);

    // input.init();
    // output.init();
    init_timers();

    hw.PrintLine("%s Successfully initialized peripherals", LOG_LABEL);

    I2CHandle i2c_handle;
    hw.Print("%s Init I2C Port 1: ", LOG_LABEL);
    if (digipot::init(i2c_handle) == daisy::I2CHandle::Result::OK) {
        hw.PrintLine("%s", LOG_RESULT[1]);
    } else {
        hw.PrintLine("%s", LOG_RESULT[0]);
        goto bad_init;
    }

    hw.Print("%s Check digipots: ", LOG_LABEL);
    if (i2c_check_addr(i2c_handle, digipot::I2C_ADDRESS) == I2CHandle::Result::OK) {
        hw.PrintLine("%s", LOG_RESULT[1]);
    } else {
        hw.PrintLine("%s", LOG_RESULT[0]);
        goto bad_init;
    }

    // Tasks:
    // - read input
    // - propagate parameters to chaotic oscillators
    // - output digital oscillator
    // - manage display

    hw.PrintLine("%s System initialized successfully", LOG_LABEL);

    while (true) {
        bool expected_pending = true;

        // Process input data if new data is available
        if (input.pending_refresh.compare_exchange_strong(expected_pending, false, std::memory_order_relaxed)) {
            input.refresh(input_data);

            std::array<uint16_t, 2> params;

            for (size_t i = 0; i < 2; i++) {
                int32_t raw_value = static_cast<int32_t>(input_data.cvs[i]) +
                                    static_cast<int32_t>(input_data.encoder_values[i]);

                constexpr uint16_t max_value = std::numeric_limits<uint16_t>::max();

                params[i] = static_cast<uint16_t>(math::clamp<int32_t>(raw_value, 0, max_value));
            }

            // TODO: map params to model-specific (float) parameters
            // m_data.remap_params(...);
            // model_data_writer.swap();
        }
    }

bad_init:
    hw.SetLed(true);
    hw.PrintLine("%s Something went wrong during the initialization", LOG_LABEL);

    // output.dac.Stop();
    // display_timer.DeInit();
    // input_timer.DeInit();

    hw.PrintLine("%s System shut down", LOG_LABEL);
    hw.DeInit();

    while (true) {
        __WFE();
    }
}

void KhaosInput::init() {
    std::array<AdcChannelConfig, AdcChannels::ADC_NUM_CHANNELS> adc_config;

    // Setup Control Voltages
    adc_config[AdcChannels::ADC_CV0].InitSingle(seed::A0);
    adc_config[AdcChannels::ADC_CV1].InitSingle(seed::A1);
    adc.Init(adc_config.data(), adc_config.size());
    adc.Start();

    encoders[0].Init(seed::D17, seed::D18, seed::D24); // input 1
    encoders[1].Init(seed::D19, seed::D20, seed::D25); // input 2

    // selezione modello: analog1, analog2 o digital
    encoders[2].Init(seed::D2, seed::D3, seed::D26);

    // selezione modello: quale digitale?
    encoders[3].Init(seed::D13, seed::D14, seed::D27);
}

void KhaosInput::refresh(KhaosInputData &data) {
    /* Encoders */
    for (size_t i = 0; i < input.encoders.size(); i++) {
        input.encoders[i].Debounce();
        int32_t increment = input.encoders[i].Increment();

        int32_t new_value =
            static_cast<int32_t>(data.encoder_values[i]) +
            increment * static_cast<int32_t>(PARAM_RESOLUTION / ROTARY_ENCODER_RESOLUTION);

        constexpr uint16_t max_value = std::numeric_limits<uint16_t>::max();

        // Clamp encoder value between 0 and (2^16 - 1)
        if (new_value < 0) {
            data.encoder_values[i] = 0;
        } else if (new_value > static_cast<int>(max_value)) {
            data.encoder_values[i] = max_value;
        } else {
            data.encoder_values[i] = static_cast<uint16_t>(new_value);
        }

        // Gather switch data
        // TODO: change to FallingEdge; do not save switch data directly,
        // (falling edges may be lost due to how TripleBuffer works),
        // but rather select here which chaotic model to use
        data.switches[i] = input.encoders[i].Pressed();
    }

    /* Control Voltages */
    data.cvs[0] = input.adc.Get(KhaosInput::ADC_CV0);
    data.cvs[1] = input.adc.Get(KhaosInput::ADC_CV1);
}

void KhaosOutput::init() {
    // DAC configuration
    DacHandle::Config dac_config;
    dac_config.chn = DacHandle::Channel::BOTH;
    dac_config.buff_state = DacHandle::BufferState::DISABLED;
    dac_config.bitdepth = DacHandle::BitDepth::BITS_12;
    dac_config.mode = DacHandle::Mode::DMA;
    dac_config.target_samplerate = OUTPUT_SAMPLE_RATE;
    dac.Init(dac_config);
    dac.Start(output_buf[0].data(), output_buf[1].data(), output_buf[0].size(),
              output_dma_callback);

    // LED configuration
    GPIO::Config led_config;
    led_config.mode = GPIO::Mode::OUTPUT;

    led_config.pin = seed::D4;
    leds[0].Init(led_config);

    led_config.pin = seed::D5;
    leds[1].Init(led_config);

    led_config.pin = seed::D6;
    leds[2].Init(led_config);
}

KhaosInputData::KhaosInputData() {
    // Initialize each encoder value at half range
    for (auto &encoder_value : encoder_values) {
        encoder_value = PARAM_RESOLUTION / 2;
    }
}

void init_timers() {
    TimerHandle::Config config;

    /* Input management timer */
    config.dir = TimerHandle::Config::CounterDir::UP;
    config.enable_irq = true; // needed for user callback
    config.periph = TimerHandle::Config::Peripheral::TIM_3;
    input_timer.Init(config);
    input_timer.SetCallback(input_timer_callback);
    input_timer.SetPrescaler(3999); // avoids overflow since the timer is 16-bit
    input_timer.SetPeriod(input_timer.GetFreq() / INPUT_SAMPLE_RATE);
    input_timer.Start();

    /* Output management timer */
    config.dir = TimerHandle::Config::CounterDir::UP;
    config.enable_irq = true; // needed for user callback
    config.periph = TimerHandle::Config::Peripheral::TIM_4;
    display_timer.Init(config);
    display_timer.SetCallback(display_refresh_callback);
    display_timer.SetPrescaler(3999); // avoids overflow since the timer is 16-bit
    display_timer.SetPeriod(input_timer.GetFreq() / DISPLAY_REFRESH_RATE);
    display_timer.Start();
}

void input_timer_callback(void *data) {
    input.pending_refresh.store(true, std::memory_order_relaxed);
}

void output_dma_callback(uint16_t **out, size_t size) {
    static ChaosOsc<math::Rossler> rossler(math::Rossler{}, math::vec3f{1.0f, 1.0f, 1.0f},
                                           static_cast<float>(OUTPUT_SAMPLE_RATE), 1.0f);
    // TODO: other models...

    // Use new data to change model parameters
    if (model_data_reader.try_swap()) {
        auto &data = model_data_reader.data();

        rossler.set_model(data.rossler);
    }

    // Generate output samples
    // TODO: remap model values; choose which model; etc...
    for (size_t i = 0; i < size; i++) {
        math::vec3f state = rossler.step();
        out[0][i] = state.x();
        out[1][i] = state.y();
    }
}

void display_refresh_callback(void *data) {
    // TODO: ...
}
