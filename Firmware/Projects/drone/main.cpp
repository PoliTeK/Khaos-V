/* Khaos-V */
/* PoliTeK 2026*/

// This file contains code specialized for the final revision of the project layout.
// To prototype new code, use prototype/prototype.cpp

#include <atomic>
#include <daisy_seed.h>

#include <array>

#include "khaos_defs.hpp"
#include "math/vecmath.hpp"
// Chaotic models
#include "math/chaos_osc.hpp"
#include "math/models.hpp"

#include "menu/menu.hpp"

#include "io/i2c_utils.hpp"
#include "io/inputs.hpp"
#include "io/outputs.hpp"

using namespace daisy;
using namespace khaos;

/* --- Configuration ---------------------------------------------------------------------------- */
constexpr bool DEBUG = true;
constexpr const char *LOG_LABEL = "[Khaos-V]";
constexpr const char *LOG_ERROR = "Error";
constexpr const char *LOG_OK    = "Success";

constexpr uint32_t INPUT_SAMPLE_RATE = 20; // per second

/// Number of samples stored in the output buffer.
constexpr size_t OUTPUT_BUFFER_SIZE = 64;
/// DAC output target sample rate.
constexpr uint32_t OUTPUT_SAMPLE_RATE = 1000; // per second

// NOTE: the audio callback is called roughly OUTPUT_SAMPLE_RATE/OUTBUT_BUFFER_SIZE times per second,
// meaning that if this value is too small there will be a noticeable latency from inputs to outputs.

/// Refresh rate of the screen.
constexpr uint32_t DISPLAY_REFRESH_RATE = 30; // per second

/* --- Definitions ------------------------------------------------------------------------------ */

/**
 * Contains all hardware handles related to outputs.
 */
struct KhaosOutput {
    DacHandle dac;
    std::array<GPIO, 3> leds;

    /// @brief Initialize DAC channels
    void init_peripherals();
};

/// @brief Initializes timers
void init_timers();

void input_timer_callback(void *data);
void output_dma_callback(uint16_t **out, size_t size);
void display_refresh_callback(void *data);

/* --- Global variables ------------------------------------------------------------------------- */

/// TIM3: 16-bit timer
TimerHandle input_timer;
/// TIM4: 16-bit timer
TimerHandle display_timer;

daisy::UI ui_handle;
daisy::UiEventQueue ui_event_queue;

static io::InputHandles input{ui_event_queue};
static KhaosOutput output;

static ModelData model_data;

// static TriBuf<DigitalModelData> model_data_sync;
// static TriBuf<DigitalModelData>::Writer model_data_writer; // owner: main
// static TriBuf<DigitalModelData>::Reader model_data_reader; // owner: output_dma_callback

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
    // model_data_writer = model_data_sync.get_writer();
    // model_data_reader = model_data_sync.get_reader();

    hw.PrintLine("%s Acquired model data handles", LOG_LABEL);

    input.init_peripherals();
    output.init_peripherals();
    init_timers();

    hw.PrintLine("%s Successfully initialized peripherals", LOG_LABEL);

    I2CHandle i2c_handle;
    hw.Print("%s Init I2C Port 1: ", LOG_LABEL);
    if (io::init_i2c(i2c_handle) == daisy::I2CHandle::Result::OK) {
        hw.PrintLine("%s", LOG_OK);
    } else {
        hw.PrintLine("%s", LOG_ERROR);
        goto bad_init;
    }

    hw.Print("%s Check digipots: ", LOG_LABEL);
    if (i2c_check_addr(i2c_handle, khaos::io::I2C_DIGIPOT_ADDRESS) == I2CHandle::Result::OK) {
        hw.PrintLine("%s", LOG_OK);
    } else {
        hw.PrintLine("%s", LOG_ERROR);
        goto bad_init;
    }

    // TODO: init display
    khaos::init_ui(ui_handle, ui_event_queue, model_data);

    // Tasks:
    // - read input
    // - propagate parameters to chaotic oscillators
    // - output digital oscillator
    // - manage display

    hw.PrintLine("%s System initialized successfully", LOG_LABEL);

    while (true) {
        // Process input data if new data is available
        if (input.pending_refresh.exchange(false, std::memory_order_acquire)) {
            // TODO: use inputs
            // auto input_data = input.read_inputs();
        }

        // Update UI and refresh display
        if (display_pending_refresh.exchange(false, std::memory_order_acquire)) {
            // TODO: manage display stuff...
            ui_handle.Process();
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

/**
 *  Inits all the output peripherals.
 *  The DAC uses the TIM6 timer, which doesn't clash with the
 *  other timers we use.
 */
void KhaosOutput::init_peripherals() {
    // DAC configuration
    DacHandle::Config dac_config;
    dac_config.chn = DacHandle::Channel::BOTH;
    // TODO: set correct driver strength
    dac_config.buff_state = DacHandle::BufferState::DISABLED; /* driver strength */
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

void init_timers() {
    TimerHandle::Config config;

    /*
        Calculations:
        - 16 bit timer => counts up to 2^16 - 1 = 65'536
        - Timer base frequency: 200kHz
          this would overflow multiple times in one second
        - With 3999 prescaler, the frequency becomes 200kHz / (1 + 3999) = 50kHz
          So, in one second, the counter goes up by 50'000, safely below the overflow limit
        - INPUT_SAMPLE_RATE might be about 50Hz (we will tune it)
          With this value, the period becomes 50kHz / 50Hz = 10'000 counts
        - DISPLAY_REFRESH_RATE might be 30Hz or even lower, but still greater than 1
          Because of this, the display timer won't overflow either.

        The timer will reset the counter after this value.
    */

    /* Input management timer */
    config.dir = TimerHandle::Config::CounterDir::UP;
    config.enable_irq = true; // needed for user callback
    config.periph = TimerHandle::Config::Peripheral::TIM_3;
    input_timer.Init(config);
    input_timer.SetCallback(input_timer_callback);
    input_timer.SetPrescaler(3999);
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
    // if (model_data_reader.try_swap()) {
    //     auto &data = model_data_reader.data();

    //     rossler.set_model(data.rossler);
    // }

    // Generate output samples
    // TODO: remap model values; choose which model; etc...
    for (size_t i = 0; i < size; i++) {
        math::vec3f state = rossler.step();

        out[0][i] = static_cast<uint16_t>(state.x());
        out[1][i] = static_cast<uint16_t>(state.y());
    }
}

void display_refresh_callback(void *data) {
    display_pending_refresh.store(true, std::memory_order_release);
}
