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
    int16_t hue{0};             // HSV Hue for touchpad mode (0-360)
    uint8_t sat{255};           // HSV Saturation (0-255)
    bool combo_active{false};   // Whether START+R2 combo is held
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

// Helper: HSV to RGB conversion (optimized integer math)
static void hsv_to_rgb(int16_t h, uint8_t s, uint8_t v, uint8_t& r, uint8_t& g, uint8_t& b)
{
    if (s == 0) { r = g = b = v; return; }

    h = h % 360;
    if (h < 0) h += 360;

    uint8_t region = h / 60;
    uint8_t remainder = (h - (region * 60)) * 255 / 60;

    uint8_t p = (v * (255 - s)) / 255;
    uint8_t q = (v * (255 - ((s * remainder) / 255))) / 255;
    uint8_t t = (v * (255 - ((s * (255 - remainder)) / 255))) / 255;

    switch (region) {
        case 0: r = v; g = t; b = p; break;
        case 1: r = q; g = v; b = p; break;
        case 2: r = p; g = v; b = t; break;
        case 3: r = p; g = q; b = v; break;
        case 4: r = t; g = p; b = v; break;
        default: r = v; g = p; b = q; break;
    }
}

// Helper: Double blink Pico LED (non-blocking)
static void led_double_blink()
{
    board_api::set_led(true);
    sleep_ms(100);
    board_api::set_led(false);
    sleep_ms(100);
    board_api::set_led(true);
    sleep_ms(100);
    board_api::set_led(false);
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

    // DS4 Lightbar control: START + CROSS = battery display, START + R2 = color selection
    if (device->controller_type == CONTROLLER_TYPE_PS4Controller && device->report_parser.set_lightbar_color != NULL) {
        LightbarSettings& lb = lightbar_[idx];
        const uint32_t now_ms = board_api::ms_since_boot();

        // Detect START + CROSS combo (battery display mode)
        const bool battery_combo = (uni_gp->misc_buttons & MISC_BUTTON_START) && (uni_gp->buttons & BUTTON_A);

        // Detect START + R2 combo (color selection mode)
        const bool color_combo = (uni_gp->misc_buttons & MISC_BUTTON_START) && (uni_gp->throttle > 200);

        if (battery_combo) {
            static bool prev_battery_combo[MAX_GAMEPADS] = {false};

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

            // Intercept combo - don't send to host
            gp_in.buttons &= ~(gamepad->MAP_BUTTON_START | gamepad->MAP_BUTTON_A);
            prev_battery_combo[idx] = true;
        } else {
            static bool prev_battery_combo[MAX_GAMEPADS] = {false};
            prev_battery_combo[idx] = false;
        }

        // Battery mode: 4s smooth fade (1s fade in, 2s hold, 1s fade out)
        if (lb.battery_mode_active) {
            uint32_t elapsed = now_ms - lb.battery_mode_start_ms;
            uint8_t r, g, b;
            get_battery_color(controller->battery, r, g, b);

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

        // Color selection mode: START + R2 + (DPAD or Left Joystick)
        else if (color_combo) {
            static uint8_t prev_dpad[MAX_GAMEPADS] = {0xFF};
            static int16_t prev_joy_x[MAX_GAMEPADS] = {0};
            static int16_t prev_joy_y[MAX_GAMEPADS] = {0};
            bool color_changed = false;
            bool joy_used = false;
            uint8_t r, g, b;

            // NOTE: Touchpad code commented out (causes Pico crashes)
            /*
            int32_t delta_x = controller->mouse.delta_x;
            int32_t delta_y = controller->mouse.delta_y;
            if (delta_x != 0 || delta_y != 0) {
                lb.hue = (lb.hue + delta_x / 4) % 360;
                if (lb.hue < 0) lb.hue += 360;
                int new_sat = lb.sat - (delta_y / 2);
                lb.sat = static_cast<uint8_t>(std::min(255, std::max(0, new_sat)));
                hsv_to_rgb(lb.hue, lb.sat, lb.brightness, r, g, b);
                set_target_color(idx, r, g, b);
                color_changed = true;
            }
            */

            // Left Joystick: same as DPAD (LEFT/RIGHT=color, UP/DOWN=brightness)
            int16_t joy_x = uni_gp->axis_x - 512;  // Center at 0
            int16_t joy_y = uni_gp->axis_y - 512;

            if (abs(joy_x) > 300 || abs(joy_y) > 300) {  // Deadzone
                if (joy_x < -300 && prev_joy_x[idx] >= -300) {
                    lb.color_index = (lb.color_index == 0) ? 11 : lb.color_index - 1;
                    color_changed = true;
                    joy_used = true;
                } else if (joy_x > 300 && prev_joy_x[idx] <= 300) {
                    lb.color_index = (lb.color_index + 1) % 12;
                    color_changed = true;
                    joy_used = true;
                } else if (joy_y < -300 && prev_joy_y[idx] >= -300) {
                    lb.brightness = std::min(255, lb.brightness + 30);  // Faster: 25->30
                    color_changed = true;
                    joy_used = true;
                } else if (joy_y > 300 && prev_joy_y[idx] <= 300) {
                    lb.brightness = std::max(10, lb.brightness - 30);
                    color_changed = true;
                    joy_used = true;
                }
                prev_joy_x[idx] = joy_x;
                prev_joy_y[idx] = joy_y;
            } else {
                prev_joy_x[idx] = 0;
                prev_joy_y[idx] = 0;
            }

            // DPAD: cycle presets (faster: triggers on every frame held, not just press)
            if (!color_changed) {
                static uint8_t dpad_repeat_delay[MAX_GAMEPADS] = {0};

                if (uni_gp->dpad == DPAD_LEFT) {
                    if (prev_dpad[idx] != DPAD_LEFT || dpad_repeat_delay[idx]++ > 3) {  // Faster: 3 frames
                        lb.color_index = (lb.color_index == 0) ? 11 : lb.color_index - 1;
                        color_changed = true;
                        dpad_repeat_delay[idx] = 0;
                    }
                } else if (uni_gp->dpad == DPAD_RIGHT) {
                    if (prev_dpad[idx] != DPAD_RIGHT || dpad_repeat_delay[idx]++ > 3) {
                        lb.color_index = (lb.color_index + 1) % 12;
                        color_changed = true;
                        dpad_repeat_delay[idx] = 0;
                    }
                } else if (uni_gp->dpad == DPAD_UP) {
                    if (prev_dpad[idx] != DPAD_UP || dpad_repeat_delay[idx]++ > 3) {
                        lb.brightness = std::min(255, lb.brightness + 30);
                        color_changed = true;
                        dpad_repeat_delay[idx] = 0;
                    }
                } else if (uni_gp->dpad == DPAD_DOWN) {
                    if (prev_dpad[idx] != DPAD_DOWN || dpad_repeat_delay[idx]++ > 3) {
                        lb.brightness = std::max(10, lb.brightness - 30);
                        color_changed = true;
                        dpad_repeat_delay[idx] = 0;
                    }
                } else {
                    dpad_repeat_delay[idx] = 0;
                }
            }

            if (color_changed) {
                calculate_color(idx, r, g, b);
                set_target_color(idx, r, g, b);
            }

            // Rumble feedback when joystick used (low intensity)
            if (joy_used) {
                Gamepad::PadOut gp_out;
                gp_out.rumble_l = 30;  // Low rumble
                gp_out.rumble_r = 30;
                gamepad->set_pad_out(gp_out);
            }

            // Intercept combo inputs
            gp_in.buttons &= ~gamepad->MAP_BUTTON_START;
            gp_in.trigger_r = 0;
            gp_in.dpad = 0;
            gp_in.joystick_lx = 0;  // Clear left joystick
            gp_in.joystick_ly = 0;

            prev_dpad[idx] = uni_gp->dpad;
            lb.combo_active = true;
        } else {
            // No combo - show normal color or low battery warning
            uint8_t battery_pct = (controller->battery * 100) / 255;

            if (battery_pct < 20 && controller->battery > 0) {
                set_target_color(idx, 50, 0, 0);  // Dim red for low battery
            } else if (!lb.combo_active) {
                uint8_t r, g, b;
                calculate_color(idx, r, g, b);
                set_target_color(idx, r, g, b);
            }

            lb.combo_active = false;
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