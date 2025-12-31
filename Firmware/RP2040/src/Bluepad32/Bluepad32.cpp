#include <atomic>
#include <cstring>
#include <functional>
#include <pico/mutex.h>
#include <pico/cyw43_arch.h>

#include "btstack_run_loop.h"
#include "uni.h"

#include "sdkconfig.h"
#include "Bluepad32/Bluepad32.h"
#include "Board/board_api.h"
#include "Board/ogxm_log.h"

#ifndef CONFIG_BLUEPAD32_PLATFORM_CUSTOM
    #error "Pico W must use BLUEPAD32_PLATFORM_CUSTOM"
#endif

static_assert((CONFIG_BLUEPAD32_MAX_DEVICES == MAX_GAMEPADS), "Mismatch between BP32 and Gamepad max devices");

namespace bluepad32 {

static constexpr uint32_t FEEDBACK_TIME_MS = 250;
static constexpr uint32_t LED_CHECK_TIME_MS = 500;

struct BTDevice {
    bool connected{false};
    Gamepad* gamepad{nullptr};
};

BTDevice bt_devices_[MAX_GAMEPADS];
btstack_timer_source_t feedback_timer_;
btstack_timer_source_t led_timer_;
bool led_timer_set_{false};
bool feedback_timer_set_{false};

// DS4 Lightbar control settings
struct LightbarSettings {
    uint8_t color_index{7};    // Current color (0-7) - Default: White
    uint8_t brightness{128};   // Brightness (0-255) - Default: Half brightness
    bool combo_active{false};  // Whether START+R2 combo is held
    uint8_t current_r{0};      // Currently applied R value
    uint8_t current_g{0};      // Currently applied G value
    uint8_t current_b{0};      // Currently applied B value
};
LightbarSettings lightbar_[MAX_GAMEPADS];

// Predefined colors for DS4 lightbar
static const uint8_t LIGHTBAR_COLORS[8][3] = {
    {255, 0, 0},    // Red
    {0, 255, 0},    // Green
    {0, 0, 255},    // Blue
    {255, 255, 0},  // Yellow
    {255, 0, 255},  // Magenta
    {0, 255, 255},  // Cyan
    {255, 128, 0},  // Orange
    {255, 255, 255} // White
};

bool any_connected()
{
    for (auto& device : bt_devices_)
    {
        if (device.connected)
        {
            return true;
        }
    }
    return false;
}

// Helper: Apply lightbar color only if it changed (avoid redundant calls)
static void apply_lightbar_color(uni_hid_device_t* device, int idx, uint8_t r, uint8_t g, uint8_t b)
{
    LightbarSettings& lb = lightbar_[idx];
    if (lb.current_r != r || lb.current_g != g || lb.current_b != b)
    {
        device->report_parser.set_lightbar_color(device, r, g, b);
        lb.current_r = r;
        lb.current_g = g;
        lb.current_b = b;
    }
}

// Helper: Calculate color from index and brightness
static void calculate_color(int idx, uint8_t& r, uint8_t& g, uint8_t& b)
{
    LightbarSettings& lb = lightbar_[idx];
    r = (LIGHTBAR_COLORS[lb.color_index][0] * lb.brightness) / 255;
    g = (LIGHTBAR_COLORS[lb.color_index][1] * lb.brightness) / 255;
    b = (LIGHTBAR_COLORS[lb.color_index][2] * lb.brightness) / 255;
}

//This solves a function pointer/crash issue with bluepad32
void set_rumble(uni_hid_device_t* bp_device, uint16_t length, uint8_t rumble_l, uint8_t rumble_r)
{
    switch (bp_device->controller_type)
    {
        case CONTROLLER_TYPE_XBoxOneController:
            uni_hid_parser_xboxone_play_dual_rumble(bp_device, 0, length + 10, rumble_l, rumble_r);
            break;
        case CONTROLLER_TYPE_AndroidController:
            if (bp_device->vendor_id == UNI_HID_PARSER_STADIA_VID && bp_device->product_id == UNI_HID_PARSER_STADIA_PID) 
            {
                uni_hid_parser_stadia_play_dual_rumble(bp_device, 0, length, rumble_l, rumble_r);
            }
            break;
        case CONTROLLER_TYPE_PSMoveController:
            uni_hid_parser_psmove_play_dual_rumble(bp_device, 0, length, rumble_l, rumble_r);
            break;
        case CONTROLLER_TYPE_PS3Controller:
            uni_hid_parser_ds3_play_dual_rumble(bp_device, 0, length, rumble_l, rumble_r);
            break;
        case CONTROLLER_TYPE_PS4Controller:
            uni_hid_parser_ds4_play_dual_rumble(bp_device, 0, length, rumble_l, rumble_r);
            break;
        case CONTROLLER_TYPE_PS5Controller:
            uni_hid_parser_ds5_play_dual_rumble(bp_device, 0, length, rumble_l, rumble_r);
            break;
        case CONTROLLER_TYPE_WiiController:
            uni_hid_parser_wii_play_dual_rumble(bp_device, 0, length, rumble_l, rumble_r);
            break;
        case CONTROLLER_TYPE_SwitchProController:
        case CONTROLLER_TYPE_SwitchJoyConRight:
        case CONTROLLER_TYPE_SwitchJoyConLeft:
            uni_hid_parser_switch_play_dual_rumble(bp_device, 0, length, rumble_l, rumble_r);
            break;
        default:
            break;
    }
}

static void send_feedback_cb(btstack_timer_source *ts)
{
    uni_hid_device_t* bp_device = nullptr;

    for (uint8_t i = 0; i < MAX_GAMEPADS; ++i)
    {
        if (!bt_devices_[i].connected || 
            !(bp_device = uni_hid_device_get_instance_for_idx(i)))
        {
            continue;
        }

        Gamepad::PadOut gp_out = bt_devices_[i].gamepad->get_pad_out();
        if (gp_out.rumble_l > 0 || gp_out.rumble_r > 0)
        {
            set_rumble(bp_device, static_cast<uint16_t>(FEEDBACK_TIME_MS), gp_out.rumble_l, gp_out.rumble_r);
        }
    }

    btstack_run_loop_set_timer(ts, FEEDBACK_TIME_MS);
    btstack_run_loop_add_timer(ts);
}

static void check_led_cb(btstack_timer_source *ts)
{
    static bool led_state = false;

    led_state = !led_state;

    board_api::set_led(any_connected() ? true : led_state);

    btstack_run_loop_set_timer(ts, LED_CHECK_TIME_MS);
    btstack_run_loop_add_timer(ts);
}

//BT Driver

static void init(int argc, const char** arg_V) {
}

static void init_complete_cb(void) {
    uni_bt_enable_new_connections_unsafe(true);
    // uni_bt_del_keys_unsafe();
    uni_property_dump_all();
}

static uni_error_t device_discovered_cb(bd_addr_t addr, const char* name, uint16_t cod, uint8_t rssi) {
    if (!((cod & UNI_BT_COD_MINOR_MASK) & UNI_BT_COD_MINOR_GAMEPAD)) {
        return UNI_ERROR_IGNORE_DEVICE;
    }
    return UNI_ERROR_SUCCESS;
}

static void device_connected_cb(uni_hid_device_t* device) {
    int idx = uni_hid_device_get_idx_for_instance(device);
    if (idx < 0 || idx >= MAX_GAMEPADS) {
        return;
    }

    // Apply default lightbar color for DS4 on connect (white @ half brightness)
    if (device->controller_type == CONTROLLER_TYPE_PS4Controller && device->report_parser.set_lightbar_color != NULL) {
        uint8_t r, g, b;
        calculate_color(idx, r, g, b);
        apply_lightbar_color(device, idx, r, g, b);
    }
}

static void device_disconnected_cb(uni_hid_device_t* device) {
    int idx = uni_hid_device_get_idx_for_instance(device);
    if (idx >= MAX_GAMEPADS || idx < 0) {
        return;
    }

    bt_devices_[idx].connected = false;
    bt_devices_[idx].gamepad->reset_pad_in();

    if (!led_timer_set_ && !any_connected()) {
        led_timer_set_ = true;
        led_timer_.process = check_led_cb;
        led_timer_.context = nullptr;
        btstack_run_loop_set_timer(&led_timer_, LED_CHECK_TIME_MS);
        btstack_run_loop_add_timer(&led_timer_);
    }
    if (feedback_timer_set_ && !any_connected()) {
        feedback_timer_set_ = false;
        btstack_run_loop_remove_timer(&feedback_timer_);
    }
}

static uni_error_t device_ready_cb(uni_hid_device_t* device) {    
    int idx = uni_hid_device_get_idx_for_instance(device);
    if (idx >= MAX_GAMEPADS || idx < 0) {
        return UNI_ERROR_SUCCESS;
    }

    bt_devices_[idx].connected = true;

    if (led_timer_set_) {
        led_timer_set_ = false;
        btstack_run_loop_remove_timer(&led_timer_);
        board_api::set_led(true);
    }
    if (!feedback_timer_set_) {
        feedback_timer_set_ = true;
        feedback_timer_.process = send_feedback_cb;
        feedback_timer_.context = nullptr;
        btstack_run_loop_set_timer(&feedback_timer_, FEEDBACK_TIME_MS);
        btstack_run_loop_add_timer(&feedback_timer_);
    }
    return UNI_ERROR_SUCCESS;
}

static void oob_event_cb(uni_platform_oob_event_t event, void* data) {
	return;
}

static void controller_data_cb(uni_hid_device_t* device, uni_controller_t* controller) {
    static uni_gamepad_t prev_uni_gp[MAX_GAMEPADS] = {};

    if (controller->klass != UNI_CONTROLLER_CLASS_GAMEPAD){
        return;
    }

    uni_gamepad_t *uni_gp = &controller->gamepad;
    int idx = uni_hid_device_get_idx_for_instance(device);

    Gamepad* gamepad = bt_devices_[idx].gamepad;
    Gamepad::PadIn gp_in;

    switch (uni_gp->dpad) 
    {
        case DPAD_UP:
            gp_in.dpad = gamepad->MAP_DPAD_UP;
            break;
        case DPAD_DOWN:
            gp_in.dpad = gamepad->MAP_DPAD_DOWN;
            break;
        case DPAD_LEFT:
            gp_in.dpad = gamepad->MAP_DPAD_LEFT;
            break;
        case DPAD_RIGHT:
            gp_in.dpad = gamepad->MAP_DPAD_RIGHT;
            break;
        case DPAD_UP | DPAD_RIGHT:
            gp_in.dpad = gamepad->MAP_DPAD_UP_RIGHT;
            break;
        case DPAD_DOWN | DPAD_RIGHT:
            gp_in.dpad = gamepad->MAP_DPAD_DOWN_RIGHT;
            break;
        case DPAD_DOWN | DPAD_LEFT:
            gp_in.dpad = gamepad->MAP_DPAD_DOWN_LEFT;
            break;
        case DPAD_UP | DPAD_LEFT:
            gp_in.dpad = gamepad->MAP_DPAD_UP_LEFT;
            break;
        default:
            break;
    }

    if (uni_gp->buttons & BUTTON_A) gp_in.buttons |= gamepad->MAP_BUTTON_A;
    if (uni_gp->buttons & BUTTON_B) gp_in.buttons |= gamepad->MAP_BUTTON_B;
    if (uni_gp->buttons & BUTTON_X) gp_in.buttons |= gamepad->MAP_BUTTON_X;
    if (uni_gp->buttons & BUTTON_Y) gp_in.buttons |= gamepad->MAP_BUTTON_Y;
    if (uni_gp->buttons & BUTTON_SHOULDER_L) gp_in.buttons |= gamepad->MAP_BUTTON_LB;
    if (uni_gp->buttons & BUTTON_SHOULDER_R) gp_in.buttons |= gamepad->MAP_BUTTON_RB;
    if (uni_gp->buttons & BUTTON_THUMB_L)    gp_in.buttons |= gamepad->MAP_BUTTON_L3;  
    if (uni_gp->buttons & BUTTON_THUMB_R)    gp_in.buttons |= gamepad->MAP_BUTTON_R3;
    if (uni_gp->misc_buttons & MISC_BUTTON_BACK)    gp_in.buttons |= gamepad->MAP_BUTTON_BACK;
    if (uni_gp->misc_buttons & MISC_BUTTON_START)   gp_in.buttons |= gamepad->MAP_BUTTON_START;
    if (uni_gp->misc_buttons & MISC_BUTTON_SYSTEM)  gp_in.buttons |= gamepad->MAP_BUTTON_SYS;

    gp_in.trigger_l = gamepad->scale_trigger_l<10>(static_cast<uint16_t>(uni_gp->brake));
    gp_in.trigger_r = gamepad->scale_trigger_r<10>(static_cast<uint16_t>(uni_gp->throttle));

    std::tie(gp_in.joystick_lx, gp_in.joystick_ly) = gamepad->scale_joystick_l<10>(uni_gp->axis_x, uni_gp->axis_y);
    std::tie(gp_in.joystick_rx, gp_in.joystick_ry) = gamepad->scale_joystick_r<10>(uni_gp->axis_rx, uni_gp->axis_ry);

    // Neutralize motion sensor data (sixaxis) - PS3 doesn't interpret these correctly
    // Set to zero to avoid fluctuating values potentially causing timing/cache issues
    gp_in.accel_x = 0;
    gp_in.accel_y = 0;
    gp_in.accel_z = 0;
    gp_in.gyro_z = 0;

    // Extract battery level (0-255, 255 = full)
    gp_in.battery = controller->battery;

    // DS4 Lightbar control via button combo: START + R2 + D-Pad
    // LEFT/RIGHT = change color, UP/DOWN = change brightness
    if (device->controller_type == CONTROLLER_TYPE_PS4Controller && device->report_parser.set_lightbar_color != NULL) {
        // Use 0xFF as sentinel (invalid dpad state - valid range is 0x00-0x0F)
        constexpr uint8_t NO_PREV_DPAD = 0xFF;
        static uint8_t prev_dpad[MAX_GAMEPADS];
        static bool prev_dpad_initialized = false;
        if (!prev_dpad_initialized) {
            std::fill_n(prev_dpad, MAX_GAMEPADS, NO_PREV_DPAD);
            prev_dpad_initialized = true;
        }
        LightbarSettings& lb = lightbar_[idx];

        // Detect START + R2 combo (R2 > 200 = pressed)
        bool combo_held = (uni_gp->misc_buttons & MISC_BUTTON_START) && (uni_gp->throttle > 200);

        uint8_t battery_pct = (controller->battery * 100) / 255;

        if (combo_held) {
            // Change color with LEFT/RIGHT (on D-pad press, not hold)
            bool color_changed = false;
            if (uni_gp->dpad == DPAD_LEFT && prev_dpad[idx] != DPAD_LEFT) {
                lb.color_index = (lb.color_index == 0) ? 7 : lb.color_index - 1;
                color_changed = true;
            } else if (uni_gp->dpad == DPAD_RIGHT && prev_dpad[idx] != DPAD_RIGHT) {
                lb.color_index = (lb.color_index + 1) % 8;
                color_changed = true;
            }

            // Change brightness with UP/DOWN (on D-pad press, not hold)
            if (uni_gp->dpad == DPAD_UP && prev_dpad[idx] != DPAD_UP) {
                lb.brightness = (lb.brightness >= 230) ? 255 : lb.brightness + 25;
                color_changed = true;
            } else if (uni_gp->dpad == DPAD_DOWN && prev_dpad[idx] != DPAD_DOWN) {
                lb.brightness = (lb.brightness <= 25) ? 0 : lb.brightness - 25;
                color_changed = true;
            }

            // Only apply if color/brightness actually changed
            if (color_changed) {
                uint8_t r, g, b;
                calculate_color(idx, r, g, b);
                apply_lightbar_color(device, idx, r, g, b);
            }

            // Intercept combo inputs - prevent them from reaching the host/PS3
            // Clear START, R2 trigger, and any DPAD to avoid accidental game inputs
            gp_in.buttons &= ~gamepad->MAP_BUTTON_START;  // Clear START
            gp_in.trigger_r = 0;                           // Clear R2 trigger
            gp_in.dpad = 0;                                // Clear D-pad

            lb.combo_active = true;
        } else {
            // Combo not held - apply appropriate color based on state

            // TODO: Charging status colors
            // Desired behavior:
            //   - Orange (255, 128, 0): Charging (cable connected + battery < 100%)
            //   - Green (0, 255, 0): Full charge (cable connected + battery == 100%)
            //   - Custom color: Not charging (wireless)
            //   - Dim red (50, 0, 0): Low battery (<20%, overrides custom)
            //
            // BLOCKED: Bluepad32 only exposes controller->battery (0-255 percentage).
            // DS4 HID reports contain cable state in Byte 30 (USB) / Byte 32 (BT), bit 4,
            // but Bluepad32 DS4 parser doesn't extract/expose it.
            //
            // Required: Modify Bluepad32's DS4 parser to expose cable_connected field
            // See: /tmp/charging_status_notes.md for implementation details

            // Only override for low battery if we have valid battery data (>0)
            // Prevents red flash on connect when battery=0 (uninitialized)
            if (battery_pct < 20 && controller->battery > 0) {
                apply_lightbar_color(device, idx, 50, 0, 0);
            } else {
                uint8_t r, g, b;
                calculate_color(idx, r, g, b);
                apply_lightbar_color(device, idx, r, g, b);
            }

            // TODO: Flash persistence disabled - causes hang on Pico 2W multicore
            // Settings persist only during current session (until reboot/reconnect)
            // Will re-enable when proper multicore flash coordination is implemented

            lb.combo_active = false;
        }

        prev_dpad[idx] = uni_gp->dpad;
    }

    gamepad->set_pad_in(gp_in);
}

const uni_property_t* get_property_cb(uni_property_idx_t idx) 
{
    return nullptr;
}

uni_platform* get_driver() 
{
    static uni_platform driver = 
    {
        .name = "OGXMiniW",
        .init = init,
        .on_init_complete = init_complete_cb,
        .on_device_discovered = device_discovered_cb,
        .on_device_connected = device_connected_cb,
        .on_device_disconnected = device_disconnected_cb,
        .on_device_ready = device_ready_cb,
        .on_controller_data = controller_data_cb,
        .get_property = get_property_cb,
        .on_oob_event = oob_event_cb,
    };
    return &driver;
}

//Public API

void run_task(Gamepad(&gamepads)[MAX_GAMEPADS])
{
    for (uint8_t i = 0; i < MAX_GAMEPADS; ++i)
    {
        bt_devices_[i].gamepad = &gamepads[i];
    }

    uni_platform_set_custom(get_driver());
    uni_init(0, nullptr);

    led_timer_set_ = true;
    led_timer_.process = check_led_cb;
    led_timer_.context = nullptr;
    btstack_run_loop_set_timer(&led_timer_, LED_CHECK_TIME_MS);
    btstack_run_loop_add_timer(&led_timer_);

    btstack_run_loop_execute();
}

} // namespace bluepad32 