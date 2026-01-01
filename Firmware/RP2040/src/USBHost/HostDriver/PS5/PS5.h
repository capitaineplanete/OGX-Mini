#ifndef _PS5_HOST_H_
#define _PS5_HOST_H_

#include <cstdint>

#include "Descriptors/PS5.h"
#include "USBHost/HostDriver/HostDriver.h"

class PS5Host : public HostDriver
{
public:
    PS5Host(uint8_t idx)
        : HostDriver(idx) {}

    void initialize(Gamepad& gamepad, uint8_t address, uint8_t instance, const uint8_t* report_desc, uint16_t desc_len) override;
    void process_report(Gamepad& gamepad, uint8_t address, uint8_t instance, const uint8_t* report, uint16_t len) override;
    bool send_feedback(Gamepad& gamepad, uint8_t address, uint8_t instance) override;

private:
    PS5::InReport prev_in_report_{};
    PS5::OutReport out_report_{};

    // Debouncing state to prevent phantom button presses
    uint32_t last_button_change_us_{0};
    static constexpr uint32_t BUTTON_DEBOUNCE_US = 8000;  // 8ms debounce window
};

#endif // _PS5_HOST_H_