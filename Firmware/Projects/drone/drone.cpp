/* Khaos-V */
/* PoliTeK 2026*/

// This file contains code specialized for the final revision of the project layout.
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

#include "sync/TriBuf.hpp"

using namespace daisy;

/* --- Configuration ---------------------------------------------------------------------------- */
constexpr bool DEBUG = true;
constexpr const char *LOG_LABEL = "[Khaos-V]";
constexpr const char *LOG_ERROR = "Error";
constexpr const char *LOG_OK    = "Success";

constexpr uint32_t INPUT_SAMPLE_RATE = 1; // per second

/// Number of samples stored in the output buffer.
constexpr size_t OUTPUT_BUFFER_SIZE = 32;
/// DAC output target sample rate.
constexpr uint32_t OUTPUT_SAMPLE_RATE = 1000; // per second

// NOTE: the audio callback is called roughly OUTPUT_SAMPLE_RATE/OUTBUT_BUFFER_SIZE times per second,
// meaning that if this value is too small there will be a noticeable latency from inputs to outputs.

/// Refresh rate of the screen.
constexpr uint32_t DISPLAY_REFRESH_RATE = 30; // per second

/// @brief Sets how many 'ticks' cover the full range.
/// Controls the resolution for encoder-controlled parameters.
constexpr uint16_t ROTARY_ENCODER_RESOLUTION = 32;
constexpr size_t PARAM_RESOLUTION = 1024;

/* --- Definitions ------------------------------------------------------------------------------ */

struct KhaosInputData;

/**
 * Contains all hardware handles related to inputs.
 */
struct KhaosInput {
    std::atomic_bool pending_refresh{false};

    AdcHandle adc;
    std::array<Encoder, 4> encoders;

    enum AdcChannels { ADC_CV0 = 0, ADC_CV1, ADC_NUM_CHANNELS };

    /// @brief Initialize ADC channels and GPIOs
    void init();
    KhaosInputData refresh(const KhaosInputData &);
};

/**
 * Contains all hardware handles related to outputs.
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

    /// Counters for how many times each switch was pressed.
    /// Using a counter is needed to avoid losing updates accidentally.
    std::array<uint32_t, 4> switches{0};

    /// Control Voltages
    std::array<uint16_t, 2> cvs{0};

    /// @brief Initialize input data with default values
    KhaosInputData();
};

// TODO: choose models
// TODO: choose a more appropriate name
struct KhaosModelData {
    enum SelectedModel { ROSSLER = 0, NUM_MODELS } selected;
    math::Rossler rossler;
    math::Halvorsen halvorsen;

    decltype(KhaosInputData::switches) switches;

    /// @brief Initialize chaotic models with default parameters
    KhaosModelData() = default;
};

/// @brief Initializes timers
void init_timers();

void input_timer_callback(void *data);
void output_dma_callback(uint16_t **out, size_t size);
void display_refresh_callback(void *data);

/// Reads new input and processes it.
/// It must be called exclusively by main(), and thus it has access to its resources.
void handle_input();

/* --- Global variables ------------------------------------------------------------------------- */

/// TIM3: 16-bit timer
TimerHandle input_timer;
/// TIM4: 16-bit timer
TimerHandle display_timer;

static KhaosInput input;
static KhaosOutput output;

static TriBuf<KhaosModelData> model_data_sync;
static TriBuf<KhaosModelData>::Writer model_data_writer; // owner: main
static TriBuf<KhaosModelData>::Reader model_data_reader; // owner: output_dma_callback

static std::array<std::array<uint16_t, OUTPUT_BUFFER_SIZE>, 2> output_buf;

static std::atomic_bool display_pending_refresh{false};

/* --- Main code -------------------------------------------------------------------------------- */

int main() {
    DaisySeed hw;

    hw.Init();
    hw.StartLog(DEBUG);
    hw.PrintLine("%s Starting initialization...", LOG_LABEL);

    // get_writer() and get_reader() can be called without issue because
    // no one has access to these yet.
    // We must ensure that at most one "process" (i.e. main / interrupt callback)
    // has access to each of them at any time.
    model_data_writer = model_data_sync.get_writer();
    model_data_reader = model_data_sync.get_reader();

    hw.PrintLine("%s Acquired model data handles", LOG_LABEL);

    input.init();
    output.init();
    init_timers();

    hw.PrintLine("%s Successfully initialized peripherals", LOG_LABEL);

    I2CHandle i2c_handle;
    hw.Print("%s Init I2C Port 1: ", LOG_LABEL);
    if (digipot::init(i2c_handle) == daisy::I2CHandle::Result::OK) {
        hw.PrintLine("%s", LOG_OK);
    } else {
        hw.PrintLine("%s", LOG_ERROR);
        goto bad_init;
    }

    hw.Print("%s Check digipots: ", LOG_LABEL);
    if (i2c_check_addr(i2c_handle, digipot::I2C_ADDRESS) == I2CHandle::Result::OK) {
        hw.PrintLine("%s", LOG_OK);
    } else {
        hw.PrintLine("%s", LOG_ERROR);
        goto bad_init;
    }

    // TODO: init display

    // Tasks:
    // - read input
    // - propagate parameters to chaotic oscillators
    // - output digital oscillator
    // - manage display

    hw.PrintLine("%s System initialized successfully", LOG_LABEL);

    while (true) {
        // Process input data if new data is available
        if (input.pending_refresh.exchange(false, std::memory_order_acquire)) {
            handle_input();
        }

        if (display_pending_refresh.exchange(false, std::memory_order_acquire)) {
            // TODO: manage display stuff...
        }

        __WFE(); // low powah
    }

bad_init:
    hw.SetLed(true);
    hw.PrintLine("%s Something went wrong during the initialization", LOG_LABEL);

    output.dac.Stop();
    display_timer.DeInit();
    input_timer.DeInit();

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

    // Model selection: analog1, analog2 or digital
    encoders[2].Init(seed::D2, seed::D3, seed::D26);

    // Digital model selection
    encoders[3].Init(seed::D13, seed::D14, seed::D27);
}

KhaosInputData KhaosInput::refresh(const KhaosInputData &prev_data) {
    auto data = prev_data;

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
        } else if (new_value > static_cast<int32_t>(max_value)) {
            data.encoder_values[i] = max_value;
        } else {
            data.encoder_values[i] = static_cast<uint16_t>(new_value);
        }

        // Gather switch data (switch is set on falling edge)
        // Increment on falling edge
        data.switches[i] += input.encoders[i].FallingEdge() ? 1 : 0;
    }

    /* Control Voltages */
    data.cvs[0] = input.adc.Get(KhaosInput::ADC_CV0);
    data.cvs[1] = input.adc.Get(KhaosInput::ADC_CV1);

    return data;
}

void handle_input() {
    // Needed to maintain a persistent state, even if the reader loses some updates
    static KhaosInputData input_data;

    input_data = input.refresh(input_data);    

    auto& model_data = model_data_writer.data();

    std::array<uint16_t, 2> params;

    for (size_t i = 0; i < params.size(); i++) {
        // sum CV and encoder value
        int32_t raw_value = static_cast<int32_t>(input_data.cvs[i]) + static_cast<int32_t>(input_data.encoder_values[i]);
        constexpr uint16_t max_value = std::numeric_limits<uint16_t>::max();

        // clamp to 16 bit unsigned int
        params[i] = static_cast<uint16_t>(math::clamp<int32_t>(raw_value, 0, max_value));
    }

    model_data.switches = input_data.switches;

    // TODO: map params to model-specific (float) parameters
    // m_data.remap_params(...);
    model_data_writer.swap();
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
    display_timer.SetPeriod(display_timer.GetFreq() / DISPLAY_REFRESH_RATE);
    display_timer.Start();
}

void input_timer_callback(void *data) {
    input.pending_refresh.store(true, std::memory_order_release);
}

void output_dma_callback(uint16_t **out, size_t size) {
    static ChaosOsc<math::Rossler> rossler(math::Rossler{}, math::vec3f{1.0f, 1.0f, 1.0f},
                                           static_cast<float>(OUTPUT_SAMPLE_RATE), 1.0f);
    // static ChaosOsc<math::Halvorsen> 
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
    display_pending_refresh.store(true, std::memory_order_release);
}
