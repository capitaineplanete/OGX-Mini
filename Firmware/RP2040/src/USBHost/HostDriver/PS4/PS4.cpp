#include <cstring>
#include <algorithm>

#include "host/usbh.h"
#include "class/hid/hid_host.h"

#include "USBHost/HostDriver/PS4/PS4.h"

void PS4Host::initialize(Gamepad& gamepad, uint8_t address, uint8_t instance, const uint8_t* report_desc, uint16_t desc_len) 
{
    out_report_.report_id = 0x05;
    out_report_.set_led = 1;
    out_report_.lightbar_blue = 0xFF / 2;

    tuh_hid_receive_report(address, instance);
}

void PS4Host::process_report(Gamepad& gamepad, uint8_t address, uint8_t instance, const uint8_t* report, uint16_t len)
{
    std::memcpy(&in_report_, report, std::min(static_cast<size_t>(len), sizeof(PS4::InReport)));
    in_report_.buttons[2] &= PS4::COUNTER_MASK;

    // Compare masked report (not raw) to prevent counter from causing false changes
    if (std::memcmp(&in_report_, &prev_in_report_, sizeof(PS4::InReport)) == 0)
    {
        tuh_hid_receive_report(address, instance);
        return;
    }

    // Debounce button changes to prevent phantom presses from hardware bounce
    bool buttons_changed = (prev_in_report_.buttons[0] != in_report_.buttons[0]) ||
                          (prev_in_report_.buttons[1] != in_report_.buttons[1]) ||
                          (prev_in_report_.buttons[2] != in_report_.buttons[2]);

    if (buttons_changed)
    {
        uint32_t now_us = time_us_32();
        uint32_t time_since_last_change = now_us - last_button_change_us_;

        // Ignore button changes within 8ms debounce window
        if (last_button_change_us_ != 0 && time_since_last_change < BUTTON_DEBOUNCE_US)
        {
            tuh_hid_receive_report(address, instance);
            return;
        }

        last_button_change_us_ = now_us;
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

    if (tuh_hid_send_report(address, instance, 0, reinterpret_cast<const uint8_t*>(&out_report_), sizeof(PS4::OutReport)))
    {
        manage_rumble(gamepad);
        return true;
    }
    return false;
}