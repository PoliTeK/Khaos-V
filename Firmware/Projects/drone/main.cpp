/* Khaos-V */
/* PoliTeK 2026*/

// This file contains code specialized for the final revision of the project layout.
// To prototype new code, use prototype/prototype.cpp

#include <atomic>

#include <algorithm>
#include <cmath>

#include <daisy_seed.h>
#include <dev/oled_ssd130x.h>

#include <array>

#include "khaos_defs.hpp"

#include "display/display.hpp"

#include "io/i2c_utils.hpp"
#include "io/inputs.hpp"
#include "io/outputs.hpp"

#include "math/vecmath.hpp"

// Chaotic models
#include "math/chaos_osc.hpp"
#include "math/models.hpp"

#include "menu/menu.hpp"

#include "param_ranges.hpp"

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

static void process_inputs(const RawInputData& in);
static void write_digipot(digipot::Wiper wiper, float normalized);
static inline uint16_t remap_to_u12(float v, float scale);

/* --- Global variables ------------------------------------------------------------------------- */

/// TIM3: 16-bit timer
TimerHandle input_timer;
/// TIM4: 16-bit timer
TimerHandle display_timer;

daisy::UI ui_handle;
daisy::UiEventQueue ui_event_queue;

static io::InputHandles input{ui_event_queue};
static KhaosOutput output;

/// Model parameters owned by the UI and by the main loop
/// Written and read ONNLY from the main-loop context
static ModelData model_data;

/// Triple buffer used to hand parameter snapshots
/// from the main loop to the DAC DMA callback
static TriBuf<ModelSnapshot> model_data_sync;
static TriBuf<ModelSnapshot>::Writer model_data_writer; // owner: main loop
static TriBuf<ModelSnapshot>::Reader model_data_reader; // owner: output_dma_callback (DAC callback)

/// I2C handle for the digipots
static daisy::I2CHandle i2c_handle; // global so that process_inputs can write to the digipots

/// Last applied encoder values to derive per-frame deltas of the cumulative increments
static int32_t last_inc[4] = {0,0,0,0};

/// Accumulators for the selection encoders (one detent = one step)
static int32_t type_pos_acc = 0, dm_pos_acc = 0;

static std::array<std::array<uint16_t, OUTPUT_BUFFER_SIZE>, 2> output_buf;

static std::atomic_bool display_pending_refresh{false};

/* --- Main code -------------------------------------------------------------------------------- */

int main() {
    DaisySeed hw;
    display::DisplayDriver display;

    hw.Init();
    hw.StartLog(DEBUG);
    hw.PrintLine("%s Starting initialization...", LOG_LABEL);

    // get_writer() and get_reader() must be called BEFORE any callback can run
    // so that at most one "process" owns each handle
    model_data_writer = model_data_sync.get_writer();
    model_data_reader = model_data_sync.get_reader();

    if(!model_data_writer || !model_data_reader){
        hw.PrintLine("%s Failed to acquire TriBuf handles", LOG_ERROR);
        goto bad_init
    }

    hw.PrintLine("%s Acquired model data handles", LOG_LABEL);

    input.init_peripherals();
    output.init_peripherals();
    init_timers();

    hw.PrintLine("%s Successfully initialized I/O peripherals", LOG_LABEL);

    //I2CHandle i2c_handle;
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

    display::init(display);
    hw.PrintLine("%s Display initialized successfully", LOG_LABEL);

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
            process_inputs(input.read_inputs());
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
 *  The DAC uses the TIM6 timer, which doesn't clash with the other timers we use.
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

/* --- Input processing (main-loop context only) ------------------------------------------------ */

/**
* Writes to a digipot wiper only when the value actually changed,
* to avoid hammering the I2C bus at every input tick.
*/

static void write_digipot(digipot::Wiper wiper, float normalized){
    static std::array<uint16_t, DIGIPOT_NUM_WIPERS> cache{
        0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF
    };

    size_t idx;

    switch{wiper} {
        case digipot::Wiper::Wiper0: idx = 0; break;
        case digipot::Wiper::Wiper1: idx = 1; break;
        case digipot::Wiper::Wiper2: idx = 2; break;
        case digipot::Wiper::Wiper3: idx = 3; break;
        default: return;
    }

    // [0, 1] -> 0..256 (inclusive) with rounding

    float scaled = normalized*static_cast<float>(DIGIPOT_FULL_SCALE);
    uint16_t code = static_cast<uint16_t>(scaled + 0.5f);

    if (code > DIGIPOT_FULL_SCALE) {
        code = DIGIPOT_FULL_SCALE;
    }

    if (cache[idx] != code) {
        if(io::set_potentiometer(i2c_handle, wiper, code) == I2CHandle::Result::OK) {
            cache[idx] = code;
        }
        // on error retry at the next tick, value was not confirmed
    }

/**
* Transforms raw input data (encoder deltas + CVs) into model parameters.
*
* - Encoders 0/1 set the USER BASE of the active model parameters;
* - CVs 0/1 add a modulation on top of the base (base + CV contribution);
* - The resulting EFFECTIVE values are published through the TriBuf snapshot
* and, for analog models, written to the digipots via I2C.
*
* Called ONLY from the main-loop context (input timer -> pending_refresh).
*/
static void process_inputs(const RawInputData& in) {
    /* ---- 1. Encoder deltas -> parameter bases ---- */
    float da = static_cast<float>(in.encoder_increments[0] - last_inc[0]) * STEP_A;
    float db = static_cast<float>(in.encoder_increments[1] - last_inc[1]) * STEP_B;
    last_inc[0] = in.encoder_increments[0];
    last_inc[1] = in.encoder_increments[1];

    /* ---- 2. Normalized CVs [0, 1] ---- */
    float cv0 = static_cast<float>(in.cvs[0]) / 65535.0f;
    float cv1 = static_cast<float>(in.cvs[1]) / 65535.0f;
    model_data.cv[0] = cv0;
    model_data.cv[1] = cv1;

    /* ---- 3. Build the snapshot with EFFECTIVE values (base + CV) ---- */
    ModelSnapshot snap;
    snap.model_type = model_data.selected_model_type;
    snap.digital_model = model_data.selected_digital_model;
    switch (model_data.selected_model_type) {
        case ModelType::Analog1: {
            auto& p = model_data.analog1_params;
            p.user_a = std::clamp(p.user_a + da, 0.0f, 1.0f);
            p.user_b = std::clamp(p.user_b + db, 0.0f, 1.0f);
            // Effective value = encoder base + CV contribution, clamped
            snap.analog1_a = std::clamp(p.user_a + CV_DEPTH * cv0, 0.0f, 1.0f);
            snap.analog1_b = std::clamp(p.user_b + CV_DEPTH * cv1, 0.0f, 1.0f);
            // Digipot writes (blocking I2C is fine here: main context, 20 Hz, cached)
            write_digipot(WIPER_ANALOG1_A, snap.analog1_a);
            write_digipot(WIPER_ANALOG1_B, snap.analog1_b);
            break;
        }
        case ModelType::Analog2: {
            auto& p = model_data.analog2_params;
            p.user_a = std::clamp(p.user_a + da, 0.0f, 1.0f);
            p.user_b = std::clamp(p.user_b + db, 0.0f, 1.0f);
            snap.analog2_a = std::clamp(p.user_a + CV_DEPTH * cv0, 0.0f, 1.0f);
            snap.analog2_b = std::clamp(p.user_b + CV_DEPTH * cv1, 0.0f, 1.0f);
            write_digipot(WIPER_ANALOG2_A, snap.analog2_a);
            write_digipot(WIPER_ANALOG2_B, snap.analog2_b);
            break;
        }
        case ModelType::Digital:{
            switch (model_data.selected_digital_model) {
                case DigitalModelId::Rossler: {
                    auto& p = model_data.rossler_params;
                    p.user_a = std::clamp(p.user_a + da, ROSSLER_A_MIN, ROSSLER_A_MAX);
                    p.user_b = std::clamp(p.user_b + db, ROSSLER_B_MIN, ROSSLER_B_MAX);
                    snap.rossler_a = std::clamp(
                    p.user_a + CV_DEPTH * cv0 * (ROSSLER_A_MAX - ROSSLER_A_MIN),
                    ROSSLER_A_MIN, ROSSLER_A_MAX);
                    snap.rossler_b = std::clamp(
                    p.user_b + CV_DEPTH * cv1 * (ROSSLER_B_MAX - ROSSLER_B_MIN),
                    ROSSLER_B_MIN, ROSSLER_B_MAX);
                    snap.rossler_c = std::clamp(p.user_c, ROSSLER_C_MIN, ROSSLER_C_MAX);
                    break;
                }
                case DigitalModelId::Halvorsen: {
                    auto& p = model_data.halvorsen_params;
                    p.user_a = std::clamp(p.user_a + da, HALVORSEN_A_MIN, HALVORSEN_A_MAX);
                    snap.halvorsen_a = std::clamp(
                    p.user_a + CV_DEPTH * cv0 * (HALVORSEN_A_MAX - HALVORSEN_A_MIN),
                    HALVORSEN_A_MIN, HALVORSEN_A_MAX);
                    break;
                }
                case DigitalModelId::Sprott: {
                    auto& p = model_data.sprott_params;
                    p.user_a = std::clamp(p.user_a + da, SPROTT_A_MIN, SPROTT_A_MAX);
                    p.user_b = std::clamp(p.user_b + db, SPROTT_B_MIN, SPROTT_B_MAX);
                    snap.sprott_a = std::clamp(
                    p.user_a + CV_DEPTH * cv0 * (SPROTT_A_MAX - SPROTT_A_MIN),
                    SPROTT_A_MIN, SPROTT_A_MAX);
                    snap.sprott_b = std::clamp(
                    p.user_b + CV_DEPTH * cv1 * (SPROTT_B_MAX - SPROTT_B_MIN),
                    SPROTT_B_MIN, SPROTT_B_MAX);
                    break;
                }
                default: break;
            }
            break;
        }
        default: break;
    }
    /* ---- 4. Publish the snapshot for the DAC callback ---- */
    if (model_data_writer) {
        model_data_writer.data() = snap;
        model_data_writer.swap();
    }
}
}

/* --- Callbacks -------------------------------------------------------------------------------- */

void input_timer_callback(void *data) {
    input.pending_refresh.store(true, std::memory_order_release);
}

/**
* Maps a centered float (roughly within +-1 after scaling) onto the
* 12-bit unsigned DAC range, centered at half scale and clamped.
*/
static inline uint16_t remap_to_u12(float v, float scale) {
    float n = v * scale * 0.5f + 2048.0f;
    return static_cast<uint16_t>(std::clamp(n, 0.0f, 4095.0f));
}

void output_dma_callback(uint16_t **out, size_t size) {
    static ChaosOsc<math::Rossler> rossler(
        math::Rossler{}, math::vec3f{1.0f, 1.0f, 1.0f},
        static_cast<float>(OUTPUT_SAMPLE_RATE), 1.0f);

    static ChaosOsc<math::Halvorsen> halvorsen(
        math::Halvorsen{}, math::vec3f{1.0f, 1.0f, 1.0f},
        static_cast<float>(OUTPUT_SAMPLE_RATE), 1.0f);

    static ChaosOsc<math::Sprott> sprott(
        math::Sprott{}, math::vec3f{1.0f, 1.0f, 1.0f},
        static_cast<float>(OUTPUT_SAMPLE_RATE), 1.0f);
    
    static ModelType cur_type = ModelType::Digital;
    static DigitalModelId cur_dm = DigitalModelId::Rossler;
    static float cur_scale = ROSSLER_SCALE;

    // Consume the newest parameter snapshot if the writer published one
    if (model_data_reader.try_swap()) {
        const ModelSnapshot& snap = model_data_reader.data();
        
        cur_type = snap.model_type;
        cur_dm = snap.digital_model;
        switch (cur_dm) {
            case DigitalModelId::Rossler: {
                rossler.get_model().a = snap.rossler_a;
                rossler.get_model().b = snap.rossler_b;
                rossler.get_model().c = snap.rossler_c;
                cur_scale = ROSSLER_SCALE;
                break;
            }
            case DigitalModelId::Halvorsen:{
                halvorsen.get_model().a = snap.halvorsen_a;
                cur_scale = HALVORSEN_SCALE;
                break;
            }
            case DigitalModelId::Sprott:{
                sprott.get_model().a = snap.sprott_a;
                sprott.get_model().b = snap.sprott_b;
                cur_scale = SPROTT_SCALE;
                break;
            }
            default: break;
        }
    }

    // Generate output samples
    for (size_t i = 0; i < size; i++) {
        math::vec3f state;
        switch (cur_dm) {
            case DigitalModelId::Rossler: {
                state = rossler.step(); 
                break;
            }
            case DigitalModelId::Halvorsen: {
                state = halvorsen.step(); 
                break;
            }
            case DigitalModelId::Sprott: {
                state = sprott.step(); 
                break;
            }
            default: {
                state = math::vec3f{}; 
                break;
            }
        }
        out[0][i] = remap_to_u12(state.x(), cur_scale);
        out[1][i] = remap_to_u12(state.y(), cur_scale);
    }
}

void display_refresh_callback(void *data) {
    display_pending_refresh.store(true, std::memory_order_release);
}