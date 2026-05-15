#include "daisy_seed.h"
#include "dev/oled_ssd130x.h"
#include <stdio.h>
#include <string.h>
#include "../math/models.hpp"

#define DISPLAY_ADDR 0x3C
#define N_MODELS 5

using namespace daisy;

// Retrieves empiric min and max values of a certain model
typedef struct model_borders {
    float xr, xl, yr, yl, zr, zl;
} model_borders_t;

/** For now I used only the continuous models.
* We also should consider to implement this enum class in models.hpp 
*/
enum class Displayed_Models {CHUA, SPROTT, ROSSLER, HALVORSEN, LORENTZ};

class SSD130X {
    public:
        SSD130X() {};
        void InitDisplay();
        void setCurrentModel(Displayed_Models md); 
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
        Displayed_Models model_number;

        /** It must follow the order of the models defined in enum class Displayed_Models.
        * The values are assigned in InitDisplay() method, and it must not be modified.
        * To find the borders of a new model, use MinMaxFinder main code
        */
        model_borders_t borders[N_MODELS];
};
