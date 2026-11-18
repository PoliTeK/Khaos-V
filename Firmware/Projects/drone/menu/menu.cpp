#include "menu.hpp"
#include "../io/inputs.hpp"
#include "../display/display.hpp"

using namespace daisy;
using namespace khaos;

// TODO: Initialize with proper display driver
static UiCanvasDescriptor canvas_descriptor;

static GraphPage graph_page{nullptr, nullptr};
static ModelListMenu model_list_menu;
static ParametersMenu parameters_menu{nullptr};

class ParamItem : public AbstractMenu::CustomItem {
    public:
        int max_num = 3;
        int which = 0;

        // do nothing; item drawing is handled by the menu
        void Draw(OneBitGraphicsDisplay &display, int, int, Rectangle, bool) override {};
        
        [[nodiscard]] bool CanBeEnteredForEditing() const override { return true; }

        void ModifyValue(int16_t increments, uint16_t steps_per_revolution, bool is_fn_button_pressed) override {
            (void) is_fn_button_pressed;
            (void) steps_per_revolution;
            which += increments;
        }
};

static ParamItem p1, p2;


bool khaos::init_ui(UI &ui, UiEventQueue& queue, ModelData& data) {
    if (graph_page.model_data != nullptr) {
        return false;
    }

    graph_page.model_data = &data;
    graph_page.model_list_menu = &model_list_menu;

    parameters_menu.model_data = &data;

    // okBttnId and cancelBttnId are unused because we use the
    // encoder swithces instead of buttons, generating
    // EncoderActivityChanged events, which are not used by
    // AbstractMenu directly.
    UI::SpecialControlIds special_ids{
        // .okBttnId = io::ModelSelection,
        // .cancelBttnId = io::DigitalScale,
        .menuEncoderId = io::ModelSelection,
        .valueEncoderId = io::DigitalScale,
    };

    ui.Init(queue, special_ids, {canvas_descriptor});

    ui.OpenPage(graph_page);
    return true;
}

GraphPage::GraphPage(ModelData *data_ref, ModelListMenu *model_list_menu)
    : model_list_menu(model_list_menu)
    , model_data(data_ref)
{}

GraphPage::MenuState GraphPage::get_menu_state() {
    return menu_state;
}

bool GraphPage::OnEncoderActivityChanged(uint16_t encoder_id, bool is_currently_active) {
    if (encoder_id == io::ModelSelection) {
        GetParentUI()->OpenPage(*this->model_list_menu);
        return true;
    }

    return true;
}

ModelListMenu::ModelListMenu() {
    // TODO: fare attenzione che ITEM_COUNT sia corretto...
    constexpr size_t ITEM_COUNT = 3;
    std::array<daisy::AbstractMenu::ItemConfig, ITEM_COUNT> menu_items;

    // Place your chaotic models here

    menu_items[0].text = "Rossler";
    menu_items[0].type = daisy::AbstractMenu::ItemType::openUiPageItem;
    menu_items[0].asOpenUiPageItem.pageToOpen = &parameters_menu;

    menu_items[1].text = "Halvorsen";
    menu_items[1].type = daisy::AbstractMenu::ItemType::openUiPageItem;
    menu_items[1].asOpenUiPageItem.pageToOpen = &parameters_menu;

    menu_items[2].text = "Sprott";
    menu_items[2].type = daisy::AbstractMenu::ItemType::openUiPageItem;
    menu_items[2].asOpenUiPageItem.pageToOpen = &parameters_menu;

    Init(menu_items.data(), menu_items.size(), Orientation::upDownSelectLeftRightModify, false);
}

bool ModelListMenu::OnEncoderActivityChanged(uint16_t encoder_id, bool is_currently_active) {
    if (encoder_id == io::ModelSelection) {
        // TODO: select digital model under cursor
        // auto idx = GetSelectedItemIdx();

        if (is_currently_active) {
            // call proper item function
            ((AbstractMenu*) this)->OnOkayButton(1, false);

            // close AFTER we opened the parameter menu, otherwise
            // the previous call won't see the parent ui
            Close();

            return true;
        }
    } else if (encoder_id == io::DigitalScale) {
        if (is_currently_active) {
            Close();
            return true;
        }
    }

    // don't propagate events to previous pages
    return true;
}

ParametersMenu::ParametersMenu(ModelData *model_data)
    : model_data(model_data)
{
    constexpr size_t ITEM_COUNT = 3;
    std::array<AbstractMenu::ItemConfig, ITEM_COUNT> menu_items;

    // P1 mapping, P2 mapping: P1 -> A, P2 -> C
    menu_items[0].text = "P1";
    menu_items[0].type = AbstractMenu::ItemType::customItem;
    menu_items[0].asCustomItem.itemObject = &p1;

    menu_items[1].text = "P2";
    menu_items[1].type = AbstractMenu::ItemType::customItem;
    menu_items[1].asCustomItem.itemObject = &p2;
    
    menu_items[ITEM_COUNT-1].text = "Confirm";
    menu_items[ITEM_COUNT-1].type = AbstractMenu::ItemType::closeMenuItem;

    Init(menu_items.data(), menu_items.size(), Orientation::upDownSelectLeftRightModify, false);
}

void ParametersMenu::OnShow() {
    if (model_data) {

        switch(model_data->selected_digital_model) {
            case DigitalModelId::Rossler:
                p1.max_num = 3;
                break;
            case DigitalModelId::Halvorsen:
                p1.max_num = 1;
                break;
            case DigitalModelId::Sprott:
                p1.max_num = 2;
                break;
            default:
                break;
        }
    }
}

// ----------------------------------------------------------------------
// DRAWING
// ---------------------------------------------------------------------

OptionsMenu::OptionsMenu() {
    menu_items[0].type = ItemType::valueItem;
    menu_items[0].text = "Prova Opzioni";

    Init(menu_items.data(), menu_items.size(), Orientation::upDownSelectLeftRightModify, false);
}

// TODO: implement or find driver :)
void GraphPage::Draw(const UiCanvasDescriptor& canvas) {
    auto driver = static_cast<display::DisplayDriver*>(canvas.handle_);

    // TODO: implement drawing
    (void) driver;
}

void ModelListMenu::Draw(const UiCanvasDescriptor& canvas) {
    auto driver = static_cast<display::DisplayDriver*>(canvas.handle_);

    // TODO: implement drawing
    (void) driver;
}

// NOTE: not used yet
void OptionsMenu::Draw(const UiCanvasDescriptor& canvas) {
    auto driver = static_cast<display::DisplayDriver*>(canvas.handle_);

    // TODO: implement drawing
    (void) driver;
}
void ParametersMenu::Draw(UiCanvasDescriptor const& canvas) {
    auto driver = static_cast<display::DisplayDriver*>(canvas.handle_);

    // TODO: implement drawing
    (void) driver;

    // for (int i = 0; i < numItems_; i++) {
    //     // driver->DrawRect(10, 10, 50, 50, true, false);
    // }
    // if (p1.which == 3) {
        // driver->WriteChar('Z', FontDef::, true);
    // }
};


