#pragma once

#include <daisy_seed.h>
#include <dev/oled_ssd130x.h>
#include "../math/vecmath.hpp"

#define DISPLAY_ADDR 0x3C
#define N_MODELS 5

// Retrieves empiric min and max values of a certain model
using model_borders_t = struct model_borders {
    float xr, xl, yr, yl, zr, zl;
};

/** For now I used only the continuous models.
* We also should consider to implement this enum class in models.hpp 
*/
enum class DisplayedModels {CHUA = 0, SPROTT, ROSSLER, HALVORSEN, LORENTZ};

class SSD130X {
    public:
        SSD130X() = default;
        void InitDisplay();
        void setCurrentModel(DisplayedModels md); 
        void ClearAll();
        void ClearDisplay();
        void UpdateDisplay();
        void DrawPoint(math::vec3f state);
        void WriteText(int pos, const char *text);
        void SelectText(int pos);


    private:
        daisy::OledDisplay<daisy::SSD130xI2c128x64Driver> display;

        //It must follow the order of the models defined in enum class Displayed_Models
        char models_names[N_MODELS][12] = {
            "Chua",
            "Sprott",
            "Rossler",
            "Halvorsen",
            "Lorentz"
        };
        DisplayedModels model_number;

        /** It must follow the order of the models defined in enum class Displayed_Models.
        * The values are assigned in InitDisplay() method, and it must not be modified.
        * To find the borders of a new model, use MinMaxFinder main code
        */
        model_borders_t borders[N_MODELS];
};
