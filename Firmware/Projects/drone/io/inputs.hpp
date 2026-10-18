#pragma once

#include <array>
#include <atomic>

#include <daisy.h>

namespace khaos::io {
    /// @brief Sets how many 'ticks' cover the full range.
    /// Controls the resolution for encoder-controlled parameters.
    /// Does not necessarily match the number of ticks per full turn.
    constexpr uint16_t ROTARY_ENCODER_RANGE_TICKS = 32;

    // Use this nested structure to place the enum variants under a
    // namespace like `enum class` without giving up the implicit
    // conversion
    enum InputId : std::uint8_t {
        // TODO: rename encoder ids for clarity
        Encoder0,
        Encoder1,
        Encoder2,
        Encoder3,
    };

    constexpr inline std::array<std::uint16_t, 4> ENCODER_IDS {
        Encoder0, Encoder1,
        Encoder2, Encoder3
    };
    
    /**
     *  @brief Cumulative input data.
     *
     *  Input data coming from external hardware.
     *  This must be synchronized across interrupts and processed in order to
     *  to used to control chaotic models and other outputs.
     *
     *  The events are accumulated so that the number of times each
     *  event happens can be calculated from the difference between
     *  the current and the previous input data.
     */
    struct RawInputData {
        std::array<int32_t, 4> encoder_increments;

        /// Counters for how many times each switch was pressed.
        /// Using a counter is needed to avoid losing updates accidentally.
        std::array<uint32_t, 4> switches{0};

        /// Control Voltages
        std::array<uint16_t, 2> cvs{0};
    };

    /**
     * Contains all hardware handles related to inputs.
     */
    struct InputHandles {
        std::atomic_bool pending_refresh{false};

        daisy::AdcHandle adc;
        std::array<daisy::Encoder, 4> encoders;

        daisy::UiEventQueue& event_queue;

        enum AdcChannels { ADC_CV0 = 0, ADC_CV1, ADC_NUM_CHANNELS };

        /// Initializes the struct without initializing the peripherals.
        InputHandles(daisy::UiEventQueue&);

        /// @brief Initialize ADC channels and GPIOs
        void init_peripherals();
        RawInputData refresh(const RawInputData&);
    };

}
