#include "menu.hpp"
#include "../io/inputs.hpp"

using namespace daisy;
using namespace khaos;

// TODO: Initialize with proper display driver
static UiCanvasDescriptor canvas_descriptor;

void khaos::init_ui(UI &ui, UiEventQueue& queue, ModelData& data) {
    GraphMenu graph_menu{data};

    UI::SpecialControlIds special_ids{};

    // TODO: list canvases

    ui.Init(queue, special_ids, {});
    ui.OpenPage(graph_menu);
}

GraphMenu::GraphMenu(ModelData& data_ref)
    : selected_model_type(ModelType::Analog1)
    , selected_digital_model(DigitalModelId::Rossler)
    , model_data(data_ref)
    {}

GraphMenu::MenuState GraphMenu::get_menu_state() {
    return menu_state;
}

bool GraphMenu::OnEncoderTurned(uint16_t encoder_id, int16_t turns, uint16_t steps_per_revolution) {
    // TODO: fix
    if (encoder_id == io::ModelSelection) {
        auto current_target = static_cast<int32_t>(selected_model_type);
        auto new_target = current_target + turns;

        if (new_target < 0)
            new_target = static_cast<int32_t>(ModelType::NUM_MODEL_TYPES) - new_target;

        selected_digital_model = static_cast<DigitalModelId>(new_target % static_cast<int32_t>(ModelType::NUM_MODEL_TYPES));

        return true;
    }

    return false;
}

bool GraphMenu::OnEncoderActivityChanged(uint16_t encoder_id, bool is_currently_active) {
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
void GraphMenu::Draw(const UiCanvasDescriptor& canvas) {}

ModelListMenu::ModelListMenu() {
    MappedFloatValue v{0.0, 1.0, 0.5};

    menu_items[0].type = ItemType::valueItem;
    menu_items[0].text = "Prova";
    menu_items[0].asMappedValueItem.valueToModify = &v;

    Init(menu_items.data(), menu_items.size(), Orientation::upDownSelectLeftRightModify, false);
}

bool ModelListMenu::OnEncoderActivityChanged(uint16_t encoder_id, bool is_currently_active) {
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
void ModelListMenu::Draw(const UiCanvasDescriptor& canvas) {}

OptionsMenu::OptionsMenu() {
    menu_items[0].type = ItemType::valueItem;
    menu_items[0].text = "Prova Opzioni";

    Init(menu_items.data(), menu_items.size(), Orientation::upDownSelectLeftRightModify, false);
}

// TODO: implement or find driver :)
// void OptionsMenu::Draw(const UiCanvasDescriptor& canvas) {}
