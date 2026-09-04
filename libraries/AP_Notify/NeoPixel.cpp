/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "NeoPixel.h"

#if AP_NOTIFY_NEOPIXEL_ENABLED

#include "AP_Notify/AP_Notify.h"
#include "SRV_Channel/SRV_Channel.h"

// This limit is from the dshot driver rcout groups limit
#define AP_NOTIFY_NEOPIXEL_MAX_INSTANCES        4

// Datasheet: https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf
// 24bit msg as 3 byte GRB (not RGB) where first bit is G7, and last bit is B0
// (first) G7|G6|G5|G4|G3|G2|G1|G0|R7|R6|R5|R4|R3|R2|R1|R0|B7|B6|B5|B4|B3|B2|B1|B0 (last)

#define NEOPIXEL_LED_LOW    0x33
#define NEOPIXEL_LED_MEDIUM 0x7F
#define NEOPIXEL_LED_HIGH   0xFF
#define NEOPIXEL_LED_OFF    0x00

extern const AP_HAL::HAL& hal;

NeoPixel::NeoPixel() :
    SerialLED(NEOPIXEL_LED_OFF, NEOPIXEL_LED_HIGH, NEOPIXEL_LED_MEDIUM, NEOPIXEL_LED_LOW)
{
}

uint16_t NeoPixel::init_ports()
{
    uint16_t mask = 0;
    for (uint16_t i=0; i<AP_NOTIFY_NEOPIXEL_MAX_INSTANCES; i++) {
        const SRV_Channel::Function fn = (SRV_Channel::Function)((uint8_t)SRV_Channel::k_LED_neopixel1 + i);
        if (!SRV_Channels::function_assigned(fn)) {
            continue;
        }
        mask |= SRV_Channels::get_output_channel_mask(fn);
    }

#if AP_NOTIFY_NEOPIXEL_MASK
    // Use board-defined default mask if no servo functions assigned
    if (mask == 0) {
        mask = AP_NOTIFY_NEOPIXEL_MASK;
    }
#endif

    if (mask == 0) {
        return 0;
    }

    AP_SerialLED *led = AP_SerialLED::get_singleton();
    if (led == nullptr) {
        return 0;
    }

    for (uint16_t chan=0; chan<16; chan++) {
        if ((1U<<chan) & mask) {
            if (pNotify->get_led_type() & AP_Notify::Notify_LED_NeoPixel) {
                led->set_num_neopixel(chan+1, pNotify->get_led_len());
            } else if (pNotify->get_led_type() & AP_Notify::Notify_LED_NeoPixelRGB) {
                led->set_num_neopixel_rgb(chan+1, pNotify->get_led_len());
            }
        }
    }

    return mask;
}

bool NeoPixel::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    if (pNotify->get_custom_led_pattern_enable() != 1) {
        return SerialLED::hw_set_rgb(red, green, blue);
    }

    bool in_failsafe = (AP_Notify::flags.failsafe_radio ||
                        AP_Notify::flags.failsafe_gcs ||
                        AP_Notify::flags.failsafe_battery ||
                        AP_Notify::flags.failsafe_ekf ||
                        AP_Notify::flags.ekf_bad ||
                        AP_Notify::flags.gps_glitching ||
                        AP_Notify::flags.leak_detected ||
                        AP_Notify::flags.parachute_release ||
                        AP_Notify::flags.vehicle_lost ||
                        AP_Notify::flags.initialising ||
                        AP_Notify::flags.esc_calibration ||
                        AP_Notify::flags.compass_cal_running ||
                        AP_Notify::flags.temp_cal_running ||
                        AP_Notify::flags.save_trim);

    // intercept LEDs if drone is armed and there are no active failsafes
    if (AP_Notify::flags.armed && !in_failsafe) {
        return true;
    }
    else {
        return SerialLED::hw_set_rgb(red, green, blue);
    }
}

void NeoPixel::update()
{
    SerialLED::update();

    if (pNotify->get_custom_led_pattern_enable() != 1) {
        return;
    }

    bool in_failsafe = (AP_Notify::flags.failsafe_radio ||
                        AP_Notify::flags.failsafe_gcs ||
                        AP_Notify::flags.failsafe_battery ||
                        AP_Notify::flags.failsafe_ekf ||
                        AP_Notify::flags.ekf_bad ||
                        AP_Notify::flags.gps_glitching ||
                        AP_Notify::flags.leak_detected ||
                        AP_Notify::flags.parachute_release ||
                        AP_Notify::flags.vehicle_lost ||
                        AP_Notify::flags.initialising ||
                        AP_Notify::flags.esc_calibration ||
                        AP_Notify::flags.compass_cal_running ||
                        AP_Notify::flags.temp_cal_running ||
                        AP_Notify::flags.save_trim);

    if (AP_Notify::flags.armed && !in_failsafe) {
        AP_SerialLED *led = AP_SerialLED::get_singleton();
        if (led == nullptr) {
            return;
        }

        uint8_t count = pNotify->get_led_len(); // read NTF_LED_LEN
        if (count < 16) {
            return;
        }

        uint8_t brightness = (_green_curr > _blue_curr) ? _green_curr : _blue_curr;
        brightness = (brightness > _red_curr) ? brightness : _red_curr;

        uint32_t now = AP_HAL::millis();
        bool blink_on = (now % 1000) < 500;

        uint16_t mask = SRV_Channels::get_output_channel_mask(SRV_Channel::k_LED_neopixel1);

        for (uint8_t c = 0; c < 16; c++) {
            if ((1U << c) & mask) {
                uint8_t chan = c + 1;

                for (uint8_t i = 0; i < count; i++) {
                    // green LEDs: 0-2 and 4-6
                    if (i <= 2 || (i >= 4 && i <= 6)) {
                        led->set_RGB(chan, i, 0, brightness, 0);
                    }
                    // red LEDs: 8-10 and 12-14
                    else if ((i >= 8 && i <= 10) || (i >= 12 && i <= 14)) {
                        led->set_RGB(chan, i, brightness, 0, 0);
                    }
                    // blinking white LEDs: 3, 7, 11, 15
                    else if (i == 3 || i == 7 || i == 11 || i == 15) {
                        if (blink_on) {
                            led->set_RGB(chan, i, brightness, brightness, brightness);
                        } else {
                            led->set_RGB(chan, i, 0, 0, 0);
                        }
                    }
                    // turn off any extra LEDs
                    else {
                        led->set_RGB(chan, i, 0, 0, 0);
                    }
                }
                led->send(chan);
            }
        }
    }
}

#endif  // AP_NOTIFY_NEOPIXEL_ENABLED
