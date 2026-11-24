#pragma once
#include <array>
#include "math/vecmath.hpp"

namespace khaos {
    enum class DigitalModelId : uint8_t {
        Rossler = 0,
        Halvorsen,
        Sprott,
        NUM_DIGITAL_MODELS
    };

    enum class ModelType : uint8_t {
        Analog1 = 0, // A1
        Analog2, // A2
        Digital, // D
        NUM_MODEL_TYPES
    };

    /// Shared data between UI and sample calculation
    /// Parameter values set by the user + normalized input CVs
    struct ModelData {
        ModelType selected_model_type;

        /**
         *  The digital model that is currenly being executed.
         */
        DigitalModelId selected_digital_model;

        struct {
            math::vec3f x0;
            float user_a = 0.2, user_b = 0.2, user_c = 5.7;
        } rossler_params;

        struct {
            math::vec3f x0;
            float user_a = 1.89;
        } halvorsen_params;

        struct {
            math::vec3f x0;
            float user_a = 2.07, user_b = 1.79;
        } sprott_params;
        
        struct {
            float user_a = 0.5f; // default wiper position, [0,1]
            float user_b = 0.5f;
        }analog1_params;

        struct {
            float user_a = 0.5f;
            float user_b = 0.5f;
        }analog2_params;
        

        // TODO: normalized cv values [0, 1]
        std::array<float, 2> cv;

        // TODO: implement
        void reset();
            // reset to default values or to a preset
        // void update_params();
    };

    /// Immutable snapshot of parameters, produced by the main loop
    /// and then consumed by the DAC callback through TriBuf

    struct ModelSnapshot {
        ModelType model_type =  ModelType::Analog1;
        DigitalModelId digital_model = DigitalModelId::Rossler;

        // Effective values (encoder + CV) already clamped

        // Rossler
        float rossler_a = 0.2f, rossler_b = 0.2f, rossler_c = 5.7f;
        // Halvorsen
        float halvorsen_a = 1.89f;
        // Sprott
        float sprott_a = 2.07f, sprott_b = 1.79f;

        // Effective wiper position for the analog models
        float analog1_a = 0.5f, analog1_b = 0.5f;
        float analog2_a = 0.5f, analog2_b = 0.5f;

        /// Intrinsic frequency multiplier (unused)
        float freq_multiplier = 1.0f;
    };

}
/*
Side Note: 
the "user_" values remain the memorized base values
which are seen and modified by the menu
while inside the snapshot go the values after sum
This way the CV modulates without erasing the
encoder position and by removing the CV 
you return to the base values
*/