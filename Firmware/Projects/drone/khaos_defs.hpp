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

        // TODO: normalized cv values [0, 1]
        std::array<float, 2> cv;

        // TODO: implement
        void reset();
        // void update_params();
    };

}
