#include "DebugLogger.h"
#include "Descriptors/DeviceDescriptors.h"
#include "class/cdc/cdc.h"

// Static ring buffer instance
DebugLoggerDevice::RingBuffer DebugLoggerDevice::ring_buffer_;

// USB descriptors for CDC ACM
static const tusb_desc_device_t debug_logger_desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = 0xCAFE,
    .idProduct          = 0x4011,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

enum {
    ITF_NUM_CDC = 0,
    ITF_NUM_CDC_DATA,
    ITF_NUM_TOTAL
};

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)
#define EPNUM_CDC_NOTIF 0x81
#define EPNUM_CDC_OUT   0x02
#define EPNUM_CDC_IN    0x82

static const uint8_t debug_logger_desc_cfg[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64)
};

void DebugLoggerDevice::initialize() {
    log_event("=== DEBUG LOGGER INITIALIZED ===");
    log_event("Format: [timestamp_us] CHAN: data");
    log_event("Channels: BTN=Buttons, JOY=Joysticks, TRIG=Triggers, BAT=Battery, RUM=Rumble, FLASH=Flash, TIME=Timing, EVT=Event");
    log_event("================================");
}

void DebugLoggerDevice::process(const uint8_t idx, Gamepad& gamepad) {
    uint64_t start_us = time_us_64();

    if (!gamepad.new_pad_in() && !gamepad.new_pad_out()) {
        send_buffered_data();
        return;
    }

    auto pad_in = gamepad.get_pad_in();
    auto pad_out = gamepad.get_pad_out();

    // Check for changes and log only what changed
    if (last_state_.check_and_update(pad_in, pad_out)) {
        log_gamepad_state(pad_in, pad_out);
    }

    send_buffered_data();

    // Track performance
    uint32_t process_time_us = static_cast<uint32_t>(time_us_64() - start_us);
    if (process_time_us > max_process_time_us_) {
        max_process_time_us_ = process_time_us;

        // Log every 1000 iterations
        if (++process_count_ % 1000 == 0) {
            char perf_buf[128];
            int len = snprintf(perf_buf, sizeof(perf_buf),
                "PERF: max_process_time=%luus, iterations=%lu\n",
                max_process_time_us_, process_count_);
            log_line(perf_buf, len);
            max_process_time_us_ = 0;
        }
    }
}

bool DebugLoggerDevice::GamepadState::check_and_update(const Gamepad::PadIn& pad_in, const Gamepad::PadOut& pad_out) {
    bool changed = false;

    // Check digital inputs
    if (dpad != pad_in.dpad) {
        dpad = pad_in.dpad;
        changed = true;
    }

    if (buttons != pad_in.buttons) {
        buttons = pad_in.buttons;
        changed = true;
    }

    // Check triggers with threshold
    if (abs(static_cast<int>(trigger_l) - static_cast<int>(pad_in.trigger_l)) > TRIGGER_THRESHOLD) {
        trigger_l = pad_in.trigger_l;
        changed = true;
    }

    if (abs(static_cast<int>(trigger_r) - static_cast<int>(pad_in.trigger_r)) > TRIGGER_THRESHOLD) {
        trigger_r = pad_in.trigger_r;
        changed = true;
    }

    // Check joysticks with threshold
    if (abs(joystick_lx - pad_in.joystick_lx) > JOYSTICK_THRESHOLD) {
        joystick_lx = pad_in.joystick_lx;
        changed = true;
    }

    if (abs(joystick_ly - pad_in.joystick_ly) > JOYSTICK_THRESHOLD) {
        joystick_ly = pad_in.joystick_ly;
        changed = true;
    }

    if (abs(joystick_rx - pad_in.joystick_rx) > JOYSTICK_THRESHOLD) {
        joystick_rx = pad_in.joystick_rx;
        changed = true;
    }

    if (abs(joystick_ry - pad_in.joystick_ry) > JOYSTICK_THRESHOLD) {
        joystick_ry = pad_in.joystick_ry;
        changed = true;
    }

    // Battery
    if (battery != pad_in.battery) {
        battery = pad_in.battery;
        changed = true;
    }

    // Rumble
    if (rumble_l != pad_out.rumble_l || rumble_r != pad_out.rumble_r) {
        rumble_l = pad_out.rumble_l;
        rumble_r = pad_out.rumble_r;
        changed = true;
    }

    return changed;
}

void DebugLoggerDevice::log_gamepad_state(const Gamepad::PadIn& pad_in, const Gamepad::PadOut& pad_out) {
    char log_buf[MAX_LINE_SIZE];
    int len;

    // Log button changes
    if (last_state_.buttons != pad_in.buttons) {
        log_button_change(last_state_.buttons, pad_in.buttons);
    }

    // Log D-pad changes
    if (last_state_.dpad != pad_in.dpad) {
        log_dpad_change(last_state_.dpad, pad_in.dpad);
    }

    // Log triggers
    if (last_state_.trigger_l != pad_in.trigger_l || last_state_.trigger_r != pad_in.trigger_r) {
        log_timestamp();
        len = snprintf(log_buf, sizeof(log_buf),
            "TRIG: L=%u (0x%02X) R=%u (0x%02X)\n",
            pad_in.trigger_l, pad_in.trigger_l,
            pad_in.trigger_r, pad_in.trigger_r);
        log_line(log_buf, len);
    }

    // Log joysticks (with deadzone info)
    if (last_state_.joystick_lx != pad_in.joystick_lx || last_state_.joystick_ly != pad_in.joystick_ly ||
        last_state_.joystick_rx != pad_in.joystick_rx || last_state_.joystick_ry != pad_in.joystick_ry) {

        log_timestamp();
        len = snprintf(log_buf, sizeof(log_buf),
            "JOY: L=(%d,%d | 0x%04X,0x%04X) R=(%d,%d | 0x%04X,0x%04X)\n",
            pad_in.joystick_lx, pad_in.joystick_ly,
            static_cast<uint16_t>(pad_in.joystick_lx), static_cast<uint16_t>(pad_in.joystick_ly),
            pad_in.joystick_rx, pad_in.joystick_ry,
            static_cast<uint16_t>(pad_in.joystick_rx), static_cast<uint16_t>(pad_in.joystick_ry));
        log_line(log_buf, len);
    }

    // Log battery
    if (last_state_.battery != pad_in.battery) {
        log_timestamp();
        float battery_pct = (pad_in.battery / 255.0f) * 100.0f;
        len = snprintf(log_buf, sizeof(log_buf),
            "BAT: raw=%u (0x%02X) pct=%.1f%%\n",
            pad_in.battery, pad_in.battery, battery_pct);
        log_line(log_buf, len);
    }

    // Log rumble
    if (last_state_.rumble_l != pad_out.rumble_l || last_state_.rumble_r != pad_out.rumble_r) {
        log_timestamp();
        len = snprintf(log_buf, sizeof(log_buf),
            "RUM: L=%u (0x%02X) R=%u (0x%02X)\n",
            pad_out.rumble_l, pad_out.rumble_l,
            pad_out.rumble_r, pad_out.rumble_r);
        log_line(log_buf, len);
    }

    // Log motion sensors if changed (for Sixaxis support)
    static int16_t last_accel_x = 0, last_accel_y = 0, last_accel_z = 0, last_gyro_z = 0;
    if (pad_in.accel_x != last_accel_x || pad_in.accel_y != last_accel_y ||
        pad_in.accel_z != last_accel_z || pad_in.gyro_z != last_gyro_z) {

        log_timestamp();
        len = snprintf(log_buf, sizeof(log_buf),
            "MOTION: accel=(%d,%d,%d) gyro_z=%d\n",
            pad_in.accel_x, pad_in.accel_y, pad_in.accel_z, pad_in.gyro_z);
        log_line(log_buf, len);

        last_accel_x = pad_in.accel_x;
        last_accel_y = pad_in.accel_y;
        last_accel_z = pad_in.accel_z;
        last_gyro_z = pad_in.gyro_z;
    }
}

void DebugLoggerDevice::log_button_change(uint16_t old_buttons, uint16_t new_buttons) {
    char log_buf[MAX_LINE_SIZE];

    uint16_t pressed = new_buttons & ~old_buttons;
    uint16_t released = old_buttons & ~new_buttons;

    if (pressed) {
        log_timestamp();
        int len = snprintf(log_buf, sizeof(log_buf), "BTN+: ");

        // List all pressed buttons
        for (uint16_t mask = 1; mask != 0; mask <<= 1) {
            if (pressed & mask) {
                len += snprintf(log_buf + len, sizeof(log_buf) - len, "%s ", get_button_name(mask));
            }
        }
        len += snprintf(log_buf + len, sizeof(log_buf) - len, "(0x%04X)\n", new_buttons);
        log_line(log_buf, len);
    }

    if (released) {
        log_timestamp();
        int len = snprintf(log_buf, sizeof(log_buf), "BTN-: ");

        // List all released buttons
        for (uint16_t mask = 1; mask != 0; mask <<= 1) {
            if (released & mask) {
                len += snprintf(log_buf + len, sizeof(log_buf) - len, "%s ", get_button_name(mask));
            }
        }
        len += snprintf(log_buf + len, sizeof(log_buf) - len, "(0x%04X)\n", new_buttons);
        log_line(log_buf, len);
    }
}

void DebugLoggerDevice::log_dpad_change(uint8_t old_dpad, uint8_t new_dpad) {
    char log_buf[MAX_LINE_SIZE];
    log_timestamp();

    int len = snprintf(log_buf, sizeof(log_buf),
        "DPAD: %s -> %s (0x%02X)\n",
        get_dpad_name(old_dpad), get_dpad_name(new_dpad), new_dpad);
    log_line(log_buf, len);
}

const char* DebugLoggerDevice::get_button_name(uint16_t button_mask) {
    switch (button_mask) {
        case Gamepad::BUTTON_A:     return "A";
        case Gamepad::BUTTON_B:     return "B";
        case Gamepad::BUTTON_X:     return "X";
        case Gamepad::BUTTON_Y:     return "Y";
        case Gamepad::BUTTON_L3:    return "L3";
        case Gamepad::BUTTON_R3:    return "R3";
        case Gamepad::BUTTON_BACK:  return "BACK";
        case Gamepad::BUTTON_START: return "START";
        case Gamepad::BUTTON_LB:    return "LB";
        case Gamepad::BUTTON_RB:    return "RB";
        case Gamepad::BUTTON_SYS:   return "SYS";
        case Gamepad::BUTTON_MISC:  return "MISC";
        default:                    return "UNK";
    }
}

const char* DebugLoggerDevice::get_dpad_name(uint8_t dpad) {
    switch (dpad) {
        case Gamepad::DPAD_UP:          return "UP";
        case Gamepad::DPAD_DOWN:        return "DOWN";
        case Gamepad::DPAD_LEFT:        return "LEFT";
        case Gamepad::DPAD_RIGHT:       return "RIGHT";
        case Gamepad::DPAD_UP_LEFT:     return "UP_LEFT";
        case Gamepad::DPAD_UP_RIGHT:    return "UP_RIGHT";
        case Gamepad::DPAD_DOWN_LEFT:   return "DOWN_LEFT";
        case Gamepad::DPAD_DOWN_RIGHT:  return "DOWN_RIGHT";
        case Gamepad::DPAD_NONE:        return "NONE";
        default:                        return "INVALID";
    }
}

void DebugLoggerDevice::log_timestamp() {
    char ts_buf[32];
    uint64_t now_us = time_us_64();
    int len = snprintf(ts_buf, sizeof(ts_buf), "[%llu] ", now_us);
    log_line(ts_buf, len);
}

void DebugLoggerDevice::log_line(const char* line, size_t len) {
    if (len > 0 && len < MAX_LINE_SIZE) {
        ring_buffer_.write(line, len);
        log_count_++;
    }
}

void DebugLoggerDevice::send_buffered_data() {
    if (!tud_cdc_connected()) {
        return;
    }

    char send_buf[256];
    size_t available = ring_buffer_.read(send_buf, sizeof(send_buf));

    if (available > 0) {
        size_t sent = 0;
        while (sent < available) {
            uint32_t batch = tud_cdc_write_available();
            if (batch == 0) {
                tud_cdc_write_flush();
                break;
            }

            if (batch > (available - sent)) {
                batch = available - sent;
            }

            sent += tud_cdc_write(send_buf + sent, batch);
        }
        tud_cdc_write_flush();
    }
}

bool DebugLoggerDevice::RingBuffer::write(const char* str, size_t len) {
    if (len > space()) {
        return false;
    }

    for (size_t i = 0; i < len; i++) {
        data[write_pos] = str[i];
        write_pos = (write_pos + 1) % RING_BUFFER_SIZE;
    }

    return true;
}

size_t DebugLoggerDevice::RingBuffer::read(char* buffer, size_t max_len) {
    size_t avail = available();
    size_t to_read = (avail < max_len) ? avail : max_len;

    for (size_t i = 0; i < to_read; i++) {
        buffer[i] = data[read_pos];
        read_pos = (read_pos + 1) % RING_BUFFER_SIZE;
    }

    return to_read;
}

// Static logging API
void DebugLoggerDevice::log_flash_op(const char* operation, const char* key, size_t len) {
    char log_buf[128];
    uint64_t now_us = time_us_64();
    int log_len = snprintf(log_buf, sizeof(log_buf),
        "[%llu] FLASH: op=%s key=%s len=%u\n",
        now_us, operation, key, len);

    if (log_len > 0) {
        ring_buffer_.write(log_buf, log_len);
    }
}

void DebugLoggerDevice::log_timing(const char* operation, uint32_t duration_us) {
    char log_buf[128];
    uint64_t now_us = time_us_64();
    int log_len = snprintf(log_buf, sizeof(log_buf),
        "[%llu] TIME: %s took %luus\n",
        now_us, operation, duration_us);

    if (log_len > 0) {
        ring_buffer_.write(log_buf, log_len);
    }
}

void DebugLoggerDevice::log_event(const char* event) {
    char log_buf[128];
    uint64_t now_us = time_us_64();
    int log_len = snprintf(log_buf, sizeof(log_buf),
        "[%llu] EVT: %s\n", now_us, event);

    if (log_len > 0) {
        ring_buffer_.write(log_buf, log_len);
    }
}

// TinyUSB callbacks
uint16_t DebugLoggerDevice::get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen) {
    return 0;
}

void DebugLoggerDevice::set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize) {
}

bool DebugLoggerDevice::vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request) {
    return false;
}

const uint16_t* DebugLoggerDevice::get_descriptor_string_cb(uint8_t index, uint16_t langid) {
    static uint16_t desc_str[32];

    const char* str;
    switch (index) {
        case 0: return DeviceDescriptors::STRING_LANG;
        case 1: str = "OGX-Mini"; break;
        case 2: str = "Debug Logger"; break;
        case 3: str = "DEBUG001"; break;
        case 4: str = "Debug CDC"; break;
        default: return nullptr;
    }

    return get_string_descriptor(str, index);
}

const uint8_t* DebugLoggerDevice::get_descriptor_device_cb() {
    return reinterpret_cast<const uint8_t*>(&debug_logger_desc_device);
}

const uint8_t* DebugLoggerDevice::get_hid_descriptor_report_cb(uint8_t itf) {
    return nullptr;
}

const uint8_t* DebugLoggerDevice::get_descriptor_configuration_cb(uint8_t index) {
    return debug_logger_desc_cfg;
}

const uint8_t* DebugLoggerDevice::get_descriptor_device_qualifier_cb() {
    return nullptr;
}
