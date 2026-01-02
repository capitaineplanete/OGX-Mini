#ifndef _DEBUG_LOGGER_DEVICE_H_
#define _DEBUG_LOGGER_DEVICE_H_

#include <array>
#include <cstdio>
#include <hardware/timer.h>

#include "USBDevice/DeviceDriver/DeviceDriver.h"
#include "Gamepad/Gamepad.h"

class DebugLoggerDevice : public DeviceDriver
{
public:
    void initialize() override;
    void process(const uint8_t idx, Gamepad& gamepad) override;
    uint16_t get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen) override;
    void set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize) override;
    bool vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request) override;
    const uint16_t* get_descriptor_string_cb(uint8_t index, uint16_t langid) override;
    const uint8_t* get_descriptor_device_cb() override;
    const uint8_t* get_hid_descriptor_report_cb(uint8_t itf) override;
    const uint8_t* get_descriptor_configuration_cb(uint8_t index) override;
    const uint8_t* get_descriptor_device_qualifier_cb() override;

    // Logging API for external hooks
    static void log_flash_op(const char* operation, const char* key, size_t len);
    static void log_timing(const char* operation, uint32_t duration_us);
    static void log_event(const char* event);

private:
    // High-performance ring buffer for async logging
    static constexpr size_t RING_BUFFER_SIZE = 8192;
    static constexpr size_t MAX_LINE_SIZE = 256;

    struct RingBuffer {
        std::array<char, RING_BUFFER_SIZE> data;
        volatile size_t write_pos{0};
        volatile size_t read_pos{0};

        inline size_t available() const {
            return (write_pos >= read_pos) ? (write_pos - read_pos) : (RING_BUFFER_SIZE - read_pos + write_pos);
        }

        inline size_t space() const {
            return RING_BUFFER_SIZE - available() - 1;
        }

        bool write(const char* str, size_t len);
        size_t read(char* buffer, size_t max_len);
    };

    // State tracking for change detection (avoid repetitive logging)
    struct GamepadState {
        uint8_t dpad{0xFF};
        uint16_t buttons{0xFFFF};
        uint8_t trigger_l{0xFF};
        uint8_t trigger_r{0xFF};
        int16_t joystick_lx{INT16_MAX};
        int16_t joystick_ly{INT16_MAX};
        int16_t joystick_rx{INT16_MAX};
        int16_t joystick_ry{INT16_MAX};
        uint8_t battery{0xFF};
        uint8_t rumble_l{0xFF};
        uint8_t rumble_r{0xFF};

        // Thresholds for analog change detection (reduce noise)
        static constexpr int16_t JOYSTICK_THRESHOLD = 328;  // ~1% change
        static constexpr uint8_t TRIGGER_THRESHOLD = 3;     // ~1% change

        bool check_and_update(const Gamepad::PadIn& pad_in, const Gamepad::PadOut& pad_out);
    };

    static RingBuffer ring_buffer_;
    GamepadState last_state_;
    uint64_t last_log_time_us_{0};
    uint32_t log_count_{0};

    // Performance metrics
    uint32_t process_count_{0};
    uint32_t max_process_time_us_{0};

    // Efficient logging helpers
    void log_line(const char* line, size_t len);
    void log_timestamp();
    void log_gamepad_state(const Gamepad::PadIn& pad_in, const Gamepad::PadOut& pad_out);
    void log_button_change(uint16_t old_buttons, uint16_t new_buttons);
    void log_dpad_change(uint8_t old_dpad, uint8_t new_dpad);
    void send_buffered_data();

    // Static helpers for button names
    static const char* get_button_name(uint16_t button_mask);
    static const char* get_dpad_name(uint8_t dpad);
};

#endif // _DEBUG_LOGGER_DEVICE_H_
