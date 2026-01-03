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

// Color selection constants
static constexpr uint8_t R2_THRESHOLD = 200;
static constexpr uint8_t MIN_BRIGHTNESS = 10;
static constexpr uint8_t MAX_BRIGHTNESS = 255;
static constexpr uint8_t BRIGHTNESS_JUMP = 5;
static constexpr uint8_t DPAD_REPEAT_FRAMES = 3;
static constexpr uint32_t HOLD_THRESHOLD_MS = 2000;
static constexpr uint8_t NUM_COLORS = 12;

struct BTDevice {
    bool connected{false};
    Gamepad* gamepad{nullptr};
};

BTDevice bt_devices_[MAX_GAMEPADS];
btstack_timer_source_t feedback_timer_;
btstack_timer_source_t led_timer_;
bool led_timer_set_{false};
bool feedback_timer_set_{false};

// DS4 Lightbar control settings (consolidated for cache locality)
struct LightbarSettings {
    // Color state
    uint8_t color_index{11};    // Default: White
    uint8_t brightness{MIN_BRIGHTNESS};
    uint8_t target_r{0}, target_g{0}, target_b{0};
    uint8_t current_r{0}, current_g{0}, current_b{0};

    // Battery mode
    bool battery_mode_active{false};
    uint32_t battery_mode_start_ms{0};
    uint8_t saved_r{0}, saved_g{0}, saved_b{0};

    // Input state (consolidated - was scattered static arrays)
    bool prev_battery_combo{false};
    uint8_t prev_dpad{0};
    uint32_t dpad_hold_start{0};
    uint8_t dpad_repeat_counter{0};

    // Optimization: track if transition is active
    bool has_active_transition{false};
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

// Helper: 20% linear interpolation (optimized - no lambda overhead)
static inline uint8_t lerp_20(uint8_t current, uint8_t target) {
    int diff = target - current;
    if (diff == 0) return current;
    int step = diff / 5;  // 20% step
    if (step == 0) step = (diff > 0) ? 1 : -1;
    return current + step;
}

// Helper: Apply lightbar color with smooth transition
static void set_target_color(int idx, uint8_t r, uint8_t g, uint8_t b)
{
    LightbarSettings& lb = lightbar_[idx];
    lb.target_r = r;
    lb.target_g = g;
    lb.target_b = b;
    lb.has_active_transition = (r != lb.current_r || g != lb.current_g || b != lb.current_b);
}

// Helper: Smooth color interpolation (ONLY called when transition is active)
static void update_color_smooth(uni_hid_device_t* device, int idx)
{
    LightbarSettings& lb = lightbar_[idx];

    const uint8_t prev_r = lb.current_r;
    const uint8_t prev_g = lb.current_g;
    const uint8_t prev_b = lb.current_b;

    lb.current_r = lerp_20(lb.current_r, lb.target_r);
    lb.current_g = lerp_20(lb.current_g, lb.target_g);
    lb.current_b = lerp_20(lb.current_b, lb.target_b);

    // Check if transition complete
    lb.has_active_transition = (lb.current_r != lb.target_r ||
                                 lb.current_g != lb.target_g ||
                                 lb.current_b != lb.target_b);

    // CRITICAL: Only send USB packet if color actually changed
    if (prev_r != lb.current_r || prev_g != lb.current_g || prev_b != lb.current_b) {
        device->report_parser.set_lightbar_color(device, lb.current_r, lb.current_g, lb.current_b);
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

// Helper: Fast 8-bit multiply with scaling (avoids slow division)
// (x * y) / 255 ≈ (x * y + 128) >> 8  (ARM-optimized)
static inline uint8_t scale_brightness(uint8_t color, uint8_t brightness) {
    uint16_t result = color * brightness + 128;
    return result >> 8;
}

// Helper: Calculate color from index and brightness
static void calculate_color(int idx, uint8_t& r, uint8_t& g, uint8_t& b)
{
    LightbarSettings& lb = lightbar_[idx];
    const uint8_t* color = LIGHTBAR_COLORS[lb.color_index];
    r = scale_brightness(color[0], lb.brightness);
    g = scale_brightness(color[1], lb.brightness);
    b = scale_brightness(color[2], lb.brightness);
}

// Helper: Quick LED feedback (non-blocking - just set state, no sleep)
static void led_feedback()
{
    // No sleep - blocking kills input responsiveness!
    // LED state managed by led_timer callback
    board_api::set_led(true);
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

    // Apply default lightbar color for DS4 on connect (white at lowest brightness)
    if (device->controller_type == CONTROLLER_TYPE_PS4Controller && device->report_parser.set_lightbar_color != NULL) {
        LightbarSettings& lb = lightbar_[idx];
        lb.color_index = 11;    // White
        lb.brightness = MIN_BRIGHTNESS;

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

        // Detect combos
        const bool battery_combo = (uni_gp->misc_buttons & MISC_BUTTON_START) && (uni_gp->misc_buttons & MISC_BUTTON_BACK);
        const bool color_combo = (uni_gp->misc_buttons & MISC_BUTTON_START) && (uni_gp->throttle > R2_THRESHOLD);

        // Battery display: START + SELECT
        if (battery_combo) {
            if (!lb.prev_battery_combo) {
                led_feedback();
                lb.saved_r = lb.current_r;
                lb.saved_g = lb.current_g;
                lb.saved_b = lb.current_b;
                lb.battery_mode_active = true;
                lb.battery_mode_start_ms = now_ms;
                gp_in.battery_combo_triggered = true;
            }
            lb.prev_battery_combo = true;
        } else {
            lb.prev_battery_combo = false;
        }

        // Battery mode: 4s fade (1s in, 2s hold, 1s out) at MIN_BRIGHTNESS
        if (lb.battery_mode_active) {
            uint32_t elapsed = now_ms - lb.battery_mode_start_ms;
            uint8_t r, g, b;
            get_battery_color(controller->battery, r, g, b);

            // Scale to MIN_BRIGHTNESS once (not every frame!)
            r = scale_brightness(r, MIN_BRIGHTNESS);
            g = scale_brightness(g, MIN_BRIGHTNESS);
            b = scale_brightness(b, MIN_BRIGHTNESS);

            if (elapsed < 1000) {
                // Fade in
                uint8_t alpha = (elapsed * 255) / 1000;
                r = (lb.saved_r * (255 - alpha) + r * alpha) >> 8;
                g = (lb.saved_g * (255 - alpha) + g * alpha) >> 8;
                b = (lb.saved_b * (255 - alpha) + b * alpha) >> 8;
            } else if (elapsed >= 3000) {
                if (elapsed < 4000) {
                    // Fade out
                    uint8_t alpha = ((elapsed - 3000) * 255) / 1000;
                    r = (r * (255 - alpha) + lb.saved_r * alpha) >> 8;
                    g = (g * (255 - alpha) + lb.saved_g * alpha) >> 8;
                    b = (b * (255 - alpha) + lb.saved_b * alpha) >> 8;
                } else {
                    // Done
                    r = lb.saved_r;
                    g = lb.saved_g;
                    b = lb.saved_b;
                    lb.battery_mode_active = false;
                }
            }
            set_target_color(idx, r, g, b);
        }

        // Color selection: START + R2 + DPAD
        if (color_combo && !lb.battery_mode_active) {
            bool color_changed = false;
            const uint8_t dpad = uni_gp->dpad;

            // UP/DOWN: Brightness (DRY - no duplication)
            if (dpad == DPAD_UP || dpad == DPAD_DOWN) {
                if (lb.prev_dpad != dpad) {
                    lb.dpad_hold_start = now_ms;
                    lb.dpad_repeat_counter = 0;
                }

                if (lb.dpad_repeat_counter++ >= DPAD_REPEAT_FRAMES) {
                    const bool held = (now_ms - lb.dpad_hold_start) > HOLD_THRESHOLD_MS;
                    const int8_t step = (dpad == DPAD_UP) ? (held ? 1 : BRIGHTNESS_JUMP) : (held ? -1 : -BRIGHTNESS_JUMP);
                    const int new_brightness = lb.brightness + step;
                    lb.brightness = std::clamp(new_brightness, (int)MIN_BRIGHTNESS, (int)MAX_BRIGHTNESS);
                    color_changed = true;
                    lb.dpad_repeat_counter = 0;
                }
            }
            // LEFT/RIGHT: Color cycle (avoid modulo)
            else if (dpad == DPAD_LEFT) {
                if (lb.prev_dpad != DPAD_LEFT) {
                    lb.color_index = (lb.color_index == 0) ? (NUM_COLORS - 1) : (lb.color_index - 1);
                    color_changed = true;
                }
            } else if (dpad == DPAD_RIGHT) {
                if (lb.prev_dpad != DPAD_RIGHT) {
                    lb.color_index = (lb.color_index + 1 == NUM_COLORS) ? 0 : (lb.color_index + 1);
                    color_changed = true;
                }
            } else {
                lb.dpad_repeat_counter = 0;
            }

            if (color_changed) {
                uint8_t r, g, b;
                calculate_color(idx, r, g, b);
                set_target_color(idx, r, g, b);
            }

            lb.prev_dpad = dpad;

            // Intercept combo
            gp_in.buttons &= ~gamepad->MAP_BUTTON_START;
            gp_in.trigger_r = 0;
            gp_in.dpad = 0;
        }

        // Update smooth transitions ONLY when active
        if (lb.has_active_transition) {
            update_color_smooth(device, idx);
        }
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
