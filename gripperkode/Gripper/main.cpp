#include <cstdio>
#include <vector>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"

struct CloseEvent {
    uint32_t start_ms;
    uint32_t end_ms;
    std::vector<uint16_t> samples;
};

int main() {
    stdio_init_all();  // USB-CDC

    // ADC on GPIO27 (ADC1)
    adc_init();
    adc_gpio_init(27);
    adc_select_input(1);

    // LED on GPIO8 (status indicator)
    const uint PIN_LED = 8;
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 1);

    // PWM setup for gripper motor (slice 0)
    const uint PWM_PIN_A = 0;
    const uint PWM_PIN_B = 1;
    gpio_set_function(PWM_PIN_A, GPIO_FUNC_PWM);
    gpio_set_function(PWM_PIN_B, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(PWM_PIN_A);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 125.0f);
    pwm_config_set_wrap(&cfg, 999);
    pwm_init(slice, &cfg, true);

    // Buttons: open (GPIO9), close (GPIO12), dump (GPIO15)
    const uint BTN_OPEN  = 9;
    const uint BTN_CLOSE = 12;
    const uint BTN_DUMP  = 15;
    for (auto btn : {BTN_OPEN, BTN_CLOSE, BTN_DUMP}) {
        gpio_init(btn);
        gpio_set_dir(btn, GPIO_IN);
        gpio_pull_down(btn);  // active-high
    }

    const uint16_t THRESH     = 920;  // ADC threshold for close
    const uint    S_INTERVAL  = 5;    // ms between ADC samples
    std::vector<CloseEvent> log;

    while (true) {
        // --- Open gripper ---
        if (gpio_get(BTN_OPEN)) {
            // Drive motor to open
            pwm_set_chan_level(slice, PWM_CHAN_A, 0);
            pwm_set_chan_level(slice, PWM_CHAN_B, 200);
            sleep_ms(600);
            // Stop motor
            pwm_set_chan_level(slice, PWM_CHAN_A, 0);
            pwm_set_chan_level(slice, PWM_CHAN_B, 0);
        }
        // --- Close and log ---
        else if (gpio_get(BTN_CLOSE)) {
            CloseEvent ev;
            ev.start_ms = to_ms_since_boot(get_absolute_time());
            ev.samples.reserve(200);


            // Drive motor to close
            pwm_set_chan_level(slice, PWM_CHAN_A, 200);
            pwm_set_chan_level(slice, PWM_CHAN_B,   0);


            // just as the motor starts it has staic friction and therefor should not be able to stop
            for (int i = 0; i < 50; ++i) {
                ev.samples.push_back(adc_read());
                sleep_ms(S_INTERVAL);
            }


            // Phase 1: sample until threshold
            uint16_t v;
            do {
                v = adc_read();
                ev.samples.push_back(v);
                sleep_ms(S_INTERVAL);
            } while (v < THRESH);

            // Stop motor
            pwm_set_chan_level(slice, PWM_CHAN_A, 0);
            pwm_set_chan_level(slice, PWM_CHAN_B, 0);

            // Phase 2: take 50 extra samples after threshold
            for (int i = 0; i < 150; ++i) {
                ev.samples.push_back(adc_read());
                sleep_ms(S_INTERVAL);
            }

            ev.end_ms = to_ms_since_boot(get_absolute_time());
            log.push_back(std::move(ev));
        }

        // --- Dump log to PC ---
        else if (gpio_get(BTN_DUMP)) {
            printf("\n=== DUMPING %zu CLOSE EVENTS ===\n", log.size());
            for (size_t i = 0; i < log.size(); ++i) {
                auto &e = log[i];
                uint32_t dur = e.end_ms - e.start_ms;
                printf("Event %zu: start=%u ms, end=%u ms, duration=%u ms, samples=%zu\n",
                       i+1, e.start_ms, e.end_ms, dur, e.samples.size());
                for (auto s : e.samples) {
                    printf("%u\n", s);
                }
                printf("-------------------------------\n");
            }
            printf("=== END OF DUMP ===\n\n");
            log.clear();
            // Wait for release
            while (gpio_get(BTN_DUMP)) sleep_ms(1);
        }
        tight_loop_contents();
    }
    return 0;
}
// sudo minicom -D /dev/ttyACM0 -b 115200