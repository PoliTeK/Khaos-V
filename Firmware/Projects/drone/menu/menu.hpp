#pragma once
#include <cstdint>
#include <daisy.h>

namespace khaos {

    enum DigitalModelId : uint8_t {
        Rossler = 0,
        Halvorsen,
        NUM_DIGITAL_MODELS
    };

    enum class ModelType : uint8_t {
        Analog1 = 0, // A1
        Analog2, // A2
        Digital // D
    };

    struct DigitalModelData {
        // TODO: Rossler parameters
        // TODO: Halvorsen parameters
        // TODO: ??? parameters
    };

    /**
     *  Describes an atomic menu state update with all the relevant
     *  information.
     */
    struct MenuControlUpdate {
        /// Elapsed time (used by pop-ups).
        uint32_t time_elapsed_ms;

        // TODO: list all possible actions
        // TODO: (optional) turn into a bit mask
        bool action_up;
        bool action_down;
        bool action_left;
        bool action_right;
    };

    /**
     *  State machine which controls which menu is displayed, when,
     *  and for how long.
     */
    struct MenuControl {
        private:
            /**
             *  Currently selected model.
             *  All three model types (A1, A2, D) are executed in parallel,
             *  however only the selected model's parameters can be
             *  edited by the user (using the `P1` and `P2` encoders).
             */
            ModelType selected_model;

            /**
             *  The digital model that is currenly being executed.
             */
            DigitalModelId selected_digital_model;

            /**
             *  When switching to another digital model,
             *  all the previous model parameters are saved here.
             */
            DigitalModelData digital_data;

            enum class MenuState : uint8_t {
                NoMenu,
                ShowingModelList,
                ShowingOptions,
            };

            /**
             *   Pop-up remaining time in milliseconds.
             *   Zero if the current menu is not a pop-up.
             */
            uint32_t popup_remaining_ms = 0;
        
        public:
            /// Default initialization for the menu state machine.
            MenuControl();

            void update(MenuControlUpdate);
    };

    class ModelListMenu : public daisy::AbstractMenu {
        private:
            /// Shared menu state
            MenuControl& menu_state;

            // TODO: turn into hard-coded array containing all the relevant stuff
            daisy::AbstractMenu::ItemConfig menu_items;

            // TODO: the menu should be organized like this
            // daisy::AbstractMenu::Orientation::upDownSelectLeftRightModify

        public:
            /// Default initialization for the model list menu.
            ModelListMenu(MenuControl&);

            /// Draws the menu.
            // TODO: implement or find driver :)
            void Draw(const daisy::UiCanvasDescriptor&) override;
    };

}
