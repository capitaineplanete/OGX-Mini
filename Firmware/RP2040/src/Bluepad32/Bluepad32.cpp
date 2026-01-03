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
    uint8_t color_index{11};    // Current color - Default: White
    uint8_t brightness{10};     // Brightness (0-255) - Default: Minimum for visibility
    bool battery_mode_active{false}; // Battery display mode (4s fade)
    uint32_t battery_mode_start_ms{0}; // When battery mode started
    uint8_t saved_r{0};         // Saved color before battery mode
    uint8_t saved_g{0};
    uint8_t saved_b{0};
    uint8_t target_r{0};        // Target color for smooth transitions
    uint8_t target_g{0};
    uint8_t target_b{0};
    uint8_t current_r{0};       // Currently applied R/G/B values
    uint8_t current_g{0};
    uint8_t current_b{0};
};
LightbarSettings lightbar_[MAX_GAMEPADS];

// Expanded color presets (12 colors)
static const uint8_t LIGHTBAR_COLORS[12][3] = {
    {255, 0, 0},    // Red
    {0, 255, 0},    // Green
    {0, 0, 255},    // Blue
    {255, 255, 0},  // Yellow
    {255, 0, 255},  // Magenta
    {0, 255, 255},  // Cyan
    {255, 128, 0},  // Orange
    {128, 0, 255},  // Purple
    {255, 192, 203},// Pink
    {0, 255, 128},  // Spring green
    {255, 64, 0},   // Red-orange
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

// Helper: Apply lightbar color with smooth transition
static void set_target_color(int idx, uint8_t r, uint8_t g, uint8_t b)
{
    LightbarSettings& lb = lightbar_[idx];
    lb.target_r = r;
    lb.target_g = g;
    lb.target_b = b;
}

// Helper: Smooth color interpolation (called each frame)
static void update_color_smooth(uni_hid_device_t* device, int idx)
{
    LightbarSettings& lb = lightbar_[idx];

    // Interpolate: move current toward target by 20% each frame (~50ms = smooth)
    auto lerp = [](uint8_t current, uint8_t target) -> uint8_t {
        int diff = target - current;
        if (diff == 0) return current;
        int step = diff / 5;  // 20% step
        if (step == 0) step = (diff > 0) ? 1 : -1;  // Min 1 step
        return current + step;
    };

    lb.current_r = lerp(lb.current_r, lb.target_r);
    lb.current_g = lerp(lb.current_g, lb.target_g);
    lb.current_b = lerp(lb.current_b, lb.target_b);

    // Apply if changed
    static uint8_t prev_r[MAX_GAMEPADS] = {0};
    static uint8_t prev_g[MAX_GAMEPADS] = {0};
    static uint8_t prev_b[MAX_GAMEPADS] = {0};

    if (prev_r[idx] != lb.current_r || prev_g[idx] != lb.current_g || prev_b[idx] != lb.current_b) {
        device->report_parser.set_lightbar_color(device, lb.current_r, lb.current_g, lb.current_b);
        prev_r[idx] = lb.current_r;
        prev_g[idx] = lb.current_g;
        prev_b[idx] = lb.current_b;
    }
}

// Helper: Apply lightbar color instantly (for backward compatibility)
static void apply_lightbar_color(uni_hid_device_t* device, int idx, uint8_t r, uint8_t g, uint8_t b)
{
    set_target_color(idx, r, g, b);
    LightbarSettings& lb = lightbar_[idx];
    lb.current_r = r;
    lb.current_g = g;
    lb.current_b = b;
    device->report_parser.set_lightbar_color(device, r, g, b);
}

// Helper: Calculate color from index and brightness
static void calculate_color(int idx, uint8_t& r, uint8_t& g, uint8_t& b)
{
    LightbarSettings& lb = lightbar_[idx];
    r = (LIGHTBAR_COLORS[lb.color_index][0] * lb.brightness) / 255;
    g = (LIGHTBAR_COLORS[lb.color_index][1] * lb.brightness) / 255;
    b = (LIGHTBAR_COLORS[lb.color_index][2] * lb.brightness) / 255;
}

// Helper: Double blink Pico LED (ends in ON state, matches mode selection pattern)
static void led_double_blink()
{
    board_api::set_led(false);
    sleep_ms(100);
    board_api::set_led(true);
    sleep_ms(100);
    board_api::set_led(false);
    sleep_ms(100);
    board_api::set_led(true);  // End in ON state
}

// Helper: Get battery color (green/orange/red based on level)
static void get_battery_color(uint8_t battery, uint8_t& r, uint8_t& g, uint8_t& b)
{
    uint8_t pct = (battery * 100) / 255;
    if (pct > 60) {
        r = 0; g = 255; b = 0;  // Green
    } else if (pct > 20) {
        r = 255; g = 128; b = 0;  // Orange
    } else {
        r = 255; g = 0; b = 0;  // Red
    }
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

    // DS4 Lightbar control: START + SELECT = battery display, START + R2 = color selection
    if (device->controller_type == CONTROLLER_TYPE_PS4Controller && device->report_parser.set_lightbar_color != NULL) {
        LightbarSettings& lb = lightbar_[idx];
        const uint32_t now_ms = board_api::ms_since_boot();

        // Detect START + SELECT combo (battery display mode)
        const bool battery_combo = (uni_gp->misc_buttons & MISC_BUTTON_START) && (uni_gp->misc_buttons & MISC_BUTTON_BACK);
        static bool prev_battery_combo[MAX_GAMEPADS] = {false};

        // Detect START + R2 combo (color selection mode)
        const bool color_combo = (uni_gp->misc_buttons & MISC_BUTTON_START) && (uni_gp->throttle > 200);

        if (battery_combo) {
            if (!prev_battery_combo[idx]) {
                // Combo just triggered - activate 4s battery fade
                led_double_blink();  // Feedback: double blink Pico LED

                // Save current color
                lb.saved_r = lb.current_r;
                lb.saved_g = lb.current_g;
                lb.saved_b = lb.current_b;

                lb.battery_mode_active = true;
                lb.battery_mode_start_ms = now_ms;

                // Signal PS3Device for 30s real battery reporting
                gp_in.battery_combo_triggered = true;
            }

            // NOTE: Do NOT intercept combo - let it pass through to PS3
            prev_battery_combo[idx] = true;
        } else {
            prev_battery_combo[idx] = false;
        }

        // Battery mode: 4s smooth fade (1s fade in, 2s hold, 1s fade out) - LOWEST BRIGHTNESS
        if (lb.battery_mode_active) {
            uint32_t elapsed = now_ms - lb.battery_mode_start_ms;
            uint8_t r, g, b;
            get_battery_color(controller->battery, r, g, b);

            // Scale to lowest brightness (10/255)
            r = (r * 10) / 255;
            g = (g * 10) / 255;
            b = (b * 10) / 255;

            if (elapsed < 1000) {
                // Fade in over 1s
                uint8_t alpha = (elapsed * 255) / 1000;
                r = (lb.saved_r * (255 - alpha) + r * alpha) / 255;
                g = (lb.saved_g * (255 - alpha) + g * alpha) / 255;
                b = (lb.saved_b * (255 - alpha) + b * alpha) / 255;
            } else if (elapsed < 3000) {
                // Hold at battery color for 2s (1s-3s)
                // r, g, b already set
            } else if (elapsed < 4000) {
                // Fade out over 1s (3s-4s)
                uint8_t alpha = ((elapsed - 3000) * 255) / 1000;
                r = (r * (255 - alpha) + lb.saved_r * alpha) / 255;
                g = (g * (255 - alpha) + lb.saved_g * alpha) / 255;
                b = (b * (255 - alpha) + lb.saved_b * alpha) / 255;
            } else {
                // Done - restore saved color
                r = lb.saved_r;
                g = lb.saved_g;
                b = lb.saved_b;
                lb.battery_mode_active = false;
            }

            set_target_color(idx, r, g, b);
        }

        // Color selection mode: START + R2 + DPAD arrows
        if (color_combo && !lb.battery_mode_active) {
            static uint8_t prev_dpad[MAX_GAMEPADS] = {0};
            static uint32_t dpad_hold_start[MAX_GAMEPADS] = {0};
            static bool dpad_held[MAX_GAMEPADS] = {false};
            static uint8_t dpad_repeat_counter[MAX_GAMEPADS] = {0};
            bool color_changed = false;

            // UP/DOWN: Brightness adjustment (always fade)
            if (uni_gp->dpad == DPAD_UP) {
                if (prev_dpad[idx] != DPAD_UP) {
                    // First press: big jump (faded smoothly)
                    dpad_hold_start[idx] = now_ms;
                    dpad_held[idx] = false;
                    dpad_repeat_counter[idx] = 0;
                }

                // Repeat every 3 frames for smooth fade
                if (dpad_repeat_counter[idx]++ >= 3) {
                    if ((now_ms - dpad_hold_start[idx]) > 2000) {
                        // Held 2s: very smooth fade
                        lb.brightness = std::min(255, lb.brightness + 1);
                    } else {
                        // Quick presses: bigger jumps but still smooth
                        lb.brightness = std::min(255, lb.brightness + 5);
                    }
                    color_changed = true;
                    dpad_repeat_counter[idx] = 0;
                }
            } else if (uni_gp->dpad == DPAD_DOWN) {
                if (prev_dpad[idx] != DPAD_DOWN) {
                    // First press: big jump (faded smoothly)
                    dpad_hold_start[idx] = now_ms;
                    dpad_held[idx] = false;
                    dpad_repeat_counter[idx] = 0;
                }

                // Repeat every 3 frames for smooth fade
                if (dpad_repeat_counter[idx]++ >= 3) {
                    if ((now_ms - dpad_hold_start[idx]) > 2000) {
                        // Held 2s: very smooth fade
                        lb.brightness = std::max(10, lb.brightness - 1);
                    } else {
                        // Quick presses: bigger jumps but still smooth
                        lb.brightness = std::max(10, lb.brightness - 5);
                    }
                    color_changed = true;
                    dpad_repeat_counter[idx] = 0;
                }
            }
            // LEFT/RIGHT: Color selection (cycle through presets with fade)
            else if (uni_gp->dpad == DPAD_LEFT) {
                if (prev_dpad[idx] != DPAD_LEFT) {
                    // Cycle to previous color
                    lb.color_index = (lb.color_index == 0) ? 11 : lb.color_index - 1;
                    color_changed = true;
                }
            } else if (uni_gp->dpad == DPAD_RIGHT) {
                if (prev_dpad[idx] != DPAD_RIGHT) {
                    // Cycle to next color
                    lb.color_index = (lb.color_index + 1) % 12;
                    color_changed = true;
                }
            } else {
                // No DPAD pressed - reset counters
                dpad_held[idx] = false;
                dpad_repeat_counter[idx] = 0;
            }

            if (color_changed) {
                uint8_t r, g, b;
                calculate_color(idx, r, g, b);
                set_target_color(idx, r, g, b);
            }

            prev_dpad[idx] = uni_gp->dpad;

            // Intercept combo inputs - don't send to host
            gp_in.buttons &= ~gamepad->MAP_BUTTON_START;
            gp_in.trigger_r = 0;
            gp_in.dpad = 0;
        }

        // Always update smooth color transitions
        update_color_smooth(device, idx);
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
