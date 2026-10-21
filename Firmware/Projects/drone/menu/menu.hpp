#pragma once

#include <daisy.h>
#include "../khaos_defs.hpp"

namespace khaos {
    constexpr uint32_t MODEL_LIST_POPUP_DURATION_MS = 5000;

    enum class ModelType : uint8_t {
        Analog1 = 0, // A1
        Analog2, // A2
        Digital, // D
        NUM_MODEL_TYPES
    };

    /// Initializes the UI and provides it with a reference to the
    /// model parameters.
    void init_ui(daisy::UI &ui, daisy::UiEventQueue &queue, ModelData& data);

    class ModelListMenu : public daisy::AbstractMenu {
        private:
            // TODO: turn into hard-coded array containing all the relevant stuff
            daisy::AbstractMenu::ItemConfig menu_item;

            // TODO: the menu should be organized like this:
            // daisy::AbstractMenu::Orientation::upDownSelectLeftRightModify

        public:
            /// Default initialization for the model list menu.
            ModelListMenu();

            bool OnEncoderActivityChanged(uint16_t encoder_id, bool is_currently_active) override;

            /// Draws the menu.
            void Draw(const daisy::UiCanvasDescriptor& canvas) override;
    };

    /**
     *  @brief The main screen.
     *
     *  State machine which controls the display.
     *  It manages the waveform graph and the menu interactions, such as
     *  which menu is currently selected, options and model selections.
     *
     *  This screen is the main UI page of the Khaos-V.
     *  It displays the digital chaotic model currently under execution
     *  and relevant data associated with it;
     *  other menus are displayed on top of this page.
     */
    class GraphMenu : public daisy::UiPage {
    private:
        // TODO: create leaf pages
        ModelListMenu model_list_menu;
        // daisy::UiPage *model_list_page, *options_page;

        /**
         *  The digital model that is currenly being executed.
         */
        DigitalModelId selected_digital_model;

        /**
         *  Information about both running and suspended models,
         *  shared with other components.
         */
        ModelData& model_data;

        enum class MenuState : uint8_t {
            NoMenu, // display chaotic model
            ShowingModelList,
            ShowingOptions,
        } menu_state;

    public:
        GraphMenu(ModelData&);

        MenuState get_menu_state();

        bool OnEncoderTurned(uint16_t encoder_id, int16_t turns, uint16_t steps_per_revolution) override;
        bool OnEncoderActivityChanged(uint16_t encoder_id, bool is_currently_active) override;

        void Draw(const daisy::UiCanvasDescriptor& canvas) override;
    };
}
