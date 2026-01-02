#include <cstring>
#include <algorithm>

#include "host/usbh.h"
#include "class/hid/hid_host.h"

#include "USBHost/HostDriver/PS4/PS4.h"
#include "Board/ogxm_log.h"

void PS4Host::initialize(Gamepad& gamepad, uint8_t address, uint8_t instance, const uint8_t* report_desc, uint16_t desc_len)
{
    OGXM_USB_LOG("\n========================================\n");
    OGXM_USB_LOG("[DS4] INIT: Controller connected\n");
    OGXM_USB_LOG("[DS4] INIT: USB addr=%d inst=%d\n", address, instance);
    OGXM_USB_LOG("[DS4] INIT: Report descriptor size=%d bytes\n", desc_len);

    out_report_.report_id = 0x05;
    out_report_.set_led = 1;
    out_report_.lightbar_blue = 0xFF / 2;

    OGXM_USB_LOG("[DS4] INIT: Lightbar set to BLUE\n");
    OGXM_USB_LOG("[DS4] INIT: Ready to receive reports\n");
    OGXM_USB_LOG("========================================\n\n");

    tuh_hid_receive_report(address, instance);
}

void PS4Host::process_report(Gamepad& gamepad, uint8_t address, uint8_t instance, const uint8_t* report, uint16_t len)
{
    // DS4 sends different report IDs: 0x01 (basic) and 0x11 (extended with touchpad/gyro)
    // Only process basic report 0x01 - extended format has different byte layout
    if (len < 1 || report[0] != 0x01)
    {
        OGXM_USB_LOG("[DS4] Rejected report: ID=0x%02X len=%d (expected ID=0x01)\n",
                     len > 0 ? report[0] : 0, len);
        tuh_hid_receive_report(address, instance);
        return;
    }

    std::memcpy(&in_report_, report, std::min(static_cast<size_t>(len), sizeof(PS4::InReport)));
    in_report_.buttons[2] &= PS4::COUNTER_MASK;

    // Report processing statistics
    static uint32_t report_count = 0;
    static uint32_t stabilization_count = 0;
    static uint32_t corruption_count = 0;
    static uint32_t duplicate_count = 0;
    report_count++;

    // Only log every Nth report or when something interesting happens
    bool log_this_report = (report_count % 50 == 0); // Log every 50th report

    if (log_this_report) {
        OGXM_USB_LOG("\n[DS4] Report #%lu: L(%d,%d) R(%d,%d) Dpad=0x%X Trig(%d,%d)\n",
                     report_count,
                     in_report_.joystick_lx, in_report_.joystick_ly,
                     in_report_.joystick_rx, in_report_.joystick_ry,
                     in_report_.buttons[0] & PS4::DPAD_MASK,
                     in_report_.trigger_l, in_report_.trigger_r);
    }

    // Stabilize joystick inputs to prevent Bluetooth noise from triggering phantom movement
    // Snap values near center (126-130) to exact center (128) before processing
    // This prevents oscillating noise (127↔128↔129) from bypassing memcmp deduplication
    constexpr uint8_t CENTER = PS4::JOYSTICK_MID;
    constexpr uint8_t TOLERANCE = 2;

    auto snap_to_center = [](uint8_t value) -> uint8_t {
        // Branchless: ((value - 126) <= 4) ? 128 : value
        uint8_t diff = value - (CENTER - TOLERANCE);
        return (diff <= (TOLERANCE * 2)) ? CENTER : value;
    };

    // Track and log stabilization corrections
    uint8_t orig_lx = in_report_.joystick_lx, orig_ly = in_report_.joystick_ly;
    uint8_t orig_rx = in_report_.joystick_rx, orig_ry = in_report_.joystick_ry;

    in_report_.joystick_lx = snap_to_center(in_report_.joystick_lx);
    in_report_.joystick_ly = snap_to_center(in_report_.joystick_ly);
    in_report_.joystick_rx = snap_to_center(in_report_.joystick_rx);
    in_report_.joystick_ry = snap_to_center(in_report_.joystick_ry);

    if (orig_lx != in_report_.joystick_lx || orig_ly != in_report_.joystick_ly ||
        orig_rx != in_report_.joystick_rx || orig_ry != in_report_.joystick_ry)
    {
        stabilization_count++;
        log_this_report = true; // Always log noise corrections
        OGXM_USB_LOG("[DS4] NOISE #%lu: L(%d,%d→%d,%d) R(%d,%d→%d,%d)\n",
                     stabilization_count,
                     orig_lx, orig_ly, in_report_.joystick_lx, in_report_.joystick_ly,
                     orig_rx, orig_ry, in_report_.joystick_rx, in_report_.joystick_ry);
    }

    // Validate D-pad HAT value: only 0x00-0x08 are valid (directions + center)
    // Bluetooth corruption can cause invalid values (0x09-0x0F) → force to center
    uint8_t dpad_value = in_report_.buttons[0] & PS4::DPAD_MASK;
    if (dpad_value > PS4::Buttons0::DPAD_CENTER)
    {
        corruption_count++;
        log_this_report = true; // Always log corruption
        OGXM_USB_LOG("[DS4] CORRUPT #%lu: Dpad HAT=0x%X (invalid!) → forced CENTER\n",
                     corruption_count, dpad_value);
        in_report_.buttons[0] = (in_report_.buttons[0] & ~PS4::DPAD_MASK) | PS4::Buttons0::DPAD_CENTER;
    }

    // Compare masked report (not raw) to prevent counter from causing false changes
    if (std::memcmp(&in_report_, &prev_in_report_, sizeof(PS4::InReport)) == 0)
    {
        duplicate_count++;
        tuh_hid_receive_report(address, instance);
        return;
    }

    // Report statistics summary (every 100 reports)
    if (report_count % 100 == 0)
    {
        OGXM_USB_LOG("\n=== STATS: Rpt=%lu Noise=%lu Corrupt=%lu Dup=%lu ===\n\n",
                     report_count, stabilization_count, corruption_count, duplicate_count);
    }

    // Detect phantom L3+R3 simultaneous presses (common corruption pattern)
    if ((in_report_.buttons[1] & PS4::Buttons1::L3) && (in_report_.buttons[1] & PS4::Buttons1::R3))
    {
        log_this_report = true; // Always log phantoms
        OGXM_USB_LOG("[DS4] PHANTOM: L3+R3 pressed together!\n");
    }

    // Detect unusual button combinations that might indicate corruption
    uint8_t button_count = 0;
    uint16_t all_buttons = (in_report_.buttons[0] >> 4) | (in_report_.buttons[1] << 4) | (in_report_.buttons[2] << 12);
    for (int i = 0; i < 16; i++) {
        if (all_buttons & (1 << i)) button_count++;
    }
    if (button_count >= 6) {
        log_this_report = true; // Log unusual button combos
        OGXM_USB_LOG("[DS4] UNUSUAL: %d buttons pressed!\n", button_count);
    }

    Gamepad::PadIn gp_in;   

    switch (in_report_.buttons[0] & PS4::DPAD_MASK)
    {
        case PS4::Buttons0::DPAD_UP:
            gp_in.dpad |= gamepad.MAP_DPAD_UP;
            break;
        case PS4::Buttons0::DPAD_DOWN:
            gp_in.dpad |= gamepad.MAP_DPAD_DOWN;    
            break;
        case PS4::Buttons0::DPAD_LEFT:
            gp_in.dpad |= gamepad.MAP_DPAD_LEFT; 
            break;
        case PS4::Buttons0::DPAD_RIGHT: 
            gp_in.dpad |= gamepad.MAP_DPAD_RIGHT;
            break;
        case PS4::Buttons0::DPAD_UP_RIGHT:
            gp_in.dpad |= gamepad.MAP_DPAD_UP_RIGHT;
            break;
        case PS4::Buttons0::DPAD_RIGHT_DOWN:
            gp_in.dpad |= gamepad.MAP_DPAD_DOWN_RIGHT;
            break;
        case PS4::Buttons0::DPAD_DOWN_LEFT:
            gp_in.dpad |= gamepad.MAP_DPAD_DOWN_LEFT;
            break;
        case PS4::Buttons0::DPAD_LEFT_UP:
            gp_in.dpad |= gamepad.MAP_DPAD_UP_LEFT;
            break;
        default:
            break;
    }

    if (in_report_.buttons[0] & PS4::Buttons0::SQUARE)   gp_in.buttons |= gamepad.MAP_BUTTON_X;
    if (in_report_.buttons[0] & PS4::Buttons0::CROSS)    gp_in.buttons |= gamepad.MAP_BUTTON_A;
    if (in_report_.buttons[0] & PS4::Buttons0::CIRCLE)   gp_in.buttons |= gamepad.MAP_BUTTON_B;
    if (in_report_.buttons[0] & PS4::Buttons0::TRIANGLE) gp_in.buttons |= gamepad.MAP_BUTTON_Y; 
    if (in_report_.buttons[1] & PS4::Buttons1::L1)       gp_in.buttons |= gamepad.MAP_BUTTON_LB;
    if (in_report_.buttons[1] & PS4::Buttons1::R1)       gp_in.buttons |= gamepad.MAP_BUTTON_RB;
    if (in_report_.buttons[1] & PS4::Buttons1::L3)       gp_in.buttons |= gamepad.MAP_BUTTON_L3;
    if (in_report_.buttons[1] & PS4::Buttons1::R3)       gp_in.buttons |= gamepad.MAP_BUTTON_R3;
    if (in_report_.buttons[1] & PS4::Buttons1::SHARE)    gp_in.buttons |= gamepad.MAP_BUTTON_BACK;
    if (in_report_.buttons[1] & PS4::Buttons1::OPTIONS)  gp_in.buttons |= gamepad.MAP_BUTTON_START;
    if (in_report_.buttons[2] & PS4::Buttons2::PS)       gp_in.buttons |= gamepad.MAP_BUTTON_SYS;
    if (in_report_.buttons[2] & PS4::Buttons2::TP)       gp_in.buttons |= gamepad.MAP_BUTTON_MISC;

    gp_in.trigger_l = gamepad.scale_trigger_l(in_report_.trigger_l);
    gp_in.trigger_r = gamepad.scale_trigger_r(in_report_.trigger_r);

    std::tie(gp_in.joystick_lx, gp_in.joystick_ly) = gamepad.scale_joystick_l(in_report_.joystick_lx, in_report_.joystick_ly);
    std::tie(gp_in.joystick_rx, gp_in.joystick_ry) = gamepad.scale_joystick_r(in_report_.joystick_rx, in_report_.joystick_ry);

    // Log button presses (only if something is pressed)
    if (gp_in.buttons != 0 && log_this_report) {
        OGXM_USB_LOG("[DS4] BTN: ");
        if (gp_in.buttons & gamepad.MAP_BUTTON_X)     OGXM_USB_LOG("□ ");
        if (gp_in.buttons & gamepad.MAP_BUTTON_A)     OGXM_USB_LOG("X ");
        if (gp_in.buttons & gamepad.MAP_BUTTON_B)     OGXM_USB_LOG("O ");
        if (gp_in.buttons & gamepad.MAP_BUTTON_Y)     OGXM_USB_LOG("△ ");
        if (gp_in.buttons & gamepad.MAP_BUTTON_LB)    OGXM_USB_LOG("L1 ");
        if (gp_in.buttons & gamepad.MAP_BUTTON_RB)    OGXM_USB_LOG("R1 ");
        if (gp_in.buttons & gamepad.MAP_BUTTON_L3)    OGXM_USB_LOG("L3 ");
        if (gp_in.buttons & gamepad.MAP_BUTTON_R3)    OGXM_USB_LOG("R3 ");
        OGXM_USB_LOG("\n");
    }

    // DS4 doesn't have analog buttons, but simulate them for compatibility
    // This ensures games expecting analog button data work correctly
    gp_in.analog[gamepad.MAP_ANALOG_OFF_A]  = (gp_in.buttons & gamepad.MAP_BUTTON_A)  ? 0xFF : 0;
    gp_in.analog[gamepad.MAP_ANALOG_OFF_B]  = (gp_in.buttons & gamepad.MAP_BUTTON_B)  ? 0xFF : 0;
    gp_in.analog[gamepad.MAP_ANALOG_OFF_X]  = (gp_in.buttons & gamepad.MAP_BUTTON_X)  ? 0xFF : 0;
    gp_in.analog[gamepad.MAP_ANALOG_OFF_Y]  = (gp_in.buttons & gamepad.MAP_BUTTON_Y)  ? 0xFF : 0;
    gp_in.analog[gamepad.MAP_ANALOG_OFF_LB] = (gp_in.buttons & gamepad.MAP_BUTTON_LB) ? 0xFF : 0;
    gp_in.analog[gamepad.MAP_ANALOG_OFF_RB] = (gp_in.buttons & gamepad.MAP_BUTTON_RB) ? 0xFF : 0;

    gamepad.set_pad_in(gp_in);

    tuh_hid_receive_report(address, instance);
    std::memcpy(&prev_in_report_, &in_report_, sizeof(PS4::InReport));
}

bool PS4Host::send_feedback(Gamepad& gamepad, uint8_t address, uint8_t instance)
{
    Gamepad::PadOut gp_out = gamepad.get_pad_out();
    out_report_.motor_left = gp_out.rumble_l;
    out_report_.motor_right = gp_out.rumble_r;
    out_report_.set_rumble = (out_report_.motor_left != 0 || out_report_.motor_right != 0) ? 1 : 0;

    if (out_report_.motor_left != 0 || out_report_.motor_right != 0)
    {
        OGXM_USB_LOG("[DS4] RUMBLE: L=%d R=%d\n", out_report_.motor_left, out_report_.motor_right);
    }

    if (tuh_hid_send_report(address, instance, 0, reinterpret_cast<const uint8_t*>(&out_report_), sizeof(PS4::OutReport)))
    {
        manage_rumble(gamepad);
        return true;
    }
    return false;
}