#pragma once

namespace khaos {
    enum class DigitalModelId : uint8_t {
        Rossler = 0,
        Halvorsen,
        NUM_DIGITAL_MODELS
    };

    struct DigitalModelData {
        DigitalModelId selected_model;

        // just an example :)
        struct {
            float x0, y0;
        } rossler_params;

        // TODO: Rossler parameters
        // TODO: Halvorsen parameters
        // TODO: ??? parameters
    };

    struct ModelData {
        DigitalModelData digital;
    };

}
