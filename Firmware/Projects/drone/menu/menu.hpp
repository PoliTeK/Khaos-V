#pragma once

#include <array>

#include <daisy.h>
#include "../khaos_defs.hpp"

namespace khaos {
    // constexpr uint32_t MODEL_LIST_POPUP_DURATION_MS = 5000;

    /// Initializes the UI and provides it with a reference to the model parameters.
    bool init_ui(daisy::UI &ui, daisy::UiEventQueue &queue, ModelData& data);

    class ModelListMenu : public daisy::AbstractMenu {
        private:
            // TODO: the menu should be organized like this:
            // daisy::AbstractMenu::Orientation::upDownSelectLeftRightModify

        public:
            /// Default initialization for the model list menu.
            ModelListMenu();

            bool OnEncoderActivityChanged(uint16_t encoder_id, bool is_currently_active) override;

            /// Draws the menu.
            void Draw(const daisy::UiCanvasDescriptor& canvas) override;
    };

    class ParametersMenu : public daisy::AbstractMenu {
        private:
            ModelData *model_data;

            friend decltype(init_ui) init_ui;

        public:
            ParametersMenu(ModelData*);

            void OnShow() override;

            bool OnEncoderActivityChanged(uint16_t encoder_id, bool is_currently_active) override;

            /// Draws the menu.
            void Draw(const daisy::UiCanvasDescriptor& canvas) override;
    };

    class OptionsMenu : public daisy::AbstractMenu {
        private:
            static constexpr size_t ITEM_COUNT = 1;
            
            std::array<daisy::AbstractMenu::ItemConfig, ITEM_COUNT> menu_items;
        public:
            /// Default initialization for the model list menu.
            OptionsMenu();

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
    class GraphPage : public daisy::UiPage {
    private:
        // TODO: create leaf pages
        ModelListMenu *model_list_menu;
        // OptionListMenu option_list_menu;
        
        // daisy::UiPage *model_list_page, *options_page;

        /**
         *  Information about both running and suspended models,
         *  shared with other components.
         */
        ModelData *model_data;

        enum class MenuState : uint8_t {
            NoMenu, // display chaotic model
            ShowingModelList,
            ShowingOptions,
        } menu_state;

        friend decltype(init_ui) init_ui;

    public:
        GraphPage(ModelData*, ModelListMenu*);

        MenuState get_menu_state();

        bool OnEncoderActivityChanged(uint16_t encoder_id, bool is_currently_active) override;

        void Draw(const daisy::UiCanvasDescriptor& canvas) override;
    };
}
