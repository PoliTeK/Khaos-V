#include "menu.hpp"
#include "../io/inputs.hpp"

using namespace daisy;

// TODO: Initialize with proper display driver
static UiCanvasDescriptor canvas_descriptor;

void khaos::init_ui(daisy::UI &ui, UiEventQueue& queue, ModelData& data) {
    GraphMenu graph_menu{data};

    UI::SpecialControlIds special_ids{};

    // TODO: list canvases

    ui.Init(queue, special_ids, {});
    ui.OpenPage(graph_menu);
}

khaos::GraphMenu::GraphMenu(khaos::ModelData& data_ref)
    : selected_digital_model(DigitalModelId::Rossler)
    , model_data(data_ref)
    {}

khaos::GraphMenu::MenuState khaos::GraphMenu::get_menu_state() {
    return menu_state;
}

bool khaos::GraphMenu::OnEncoderTurned(uint16_t encoder_id, int16_t turns, uint16_t steps_per_revolution) {
    if (encoder_id == io::ModelSelection) {
        auto current_target = static_cast<int32_t>(selected_digital_model);
        auto new_target = current_target + turns;

        if (new_target < 0)
            new_target = static_cast<int32_t>(DigitalModelId::NUM_DIGITAL_MODELS) - new_target;

        selected_digital_model = static_cast<DigitalModelId>(new_target % static_cast<int32_t>(DigitalModelId::NUM_DIGITAL_MODELS));

        return true;
    }

    return false;
}

bool khaos::GraphMenu::OnEncoderActivityChanged(uint16_t encoder_id, bool is_currently_active) {
    if (encoder_id == io::ModelSelection) {
        switch (menu_state) {
            case MenuState::NoMenu:
                menu_state = MenuState::ShowingModelList;
                // TODO: show menu
                // GetParentUI()->OpenPage(model_list_page);
                break;
            case MenuState::ShowingModelList:
                menu_state = MenuState::NoMenu;
                // TODO: close menu
                // GetParentUI()->ClosePage(model_list_page);
                break;
            default:
                break;
        }
        return true;
    }

    return false;
}

// TODO: implement or find driver :)
void khaos::GraphMenu::Draw(const UiCanvasDescriptor& canvas) {}

khaos::ModelListMenu::ModelListMenu() {
    menu_item.type = ItemType::valueItem;
    menu_item.text = "Prova";

    Init(&menu_item, 1, Orientation::upDownSelectLeftRightModify, false);
}

bool khaos::ModelListMenu::OnEncoderActivityChanged(uint16_t encoder_id, bool is_currently_active) {
    if (encoder_id == io::ModelSelection) {
        // TODO: select digital model under cursor
        // auto idx = GetSelectedItemIdx();

        if (is_currently_active) {
            GetParentUI()->ClosePage(*this);
            return true;
        }
    }
    return false;
}

// TODO: implement or find driver :)
void khaos::ModelListMenu::Draw(const UiCanvasDescriptor& canvas) {}
