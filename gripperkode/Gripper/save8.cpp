#include <cstdio>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

int main() {
    stdio_init_all();         // Aktivér USB-CDC (og UART-stdio hvis slået til)

    // --- ADC setup on GPIO27 ---
    adc_init();
    adc_gpio_init(27);        // ADC1 på GPIO27
    adc_select_input(1);

    // --- LED setup on GPIO8 ---
    const uint PIN_LED = 8;
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 1);     // Tænd LED

    // --- PWM setup on GPIO0/1 (slice 0) ---
    const uint PWM_PIN_A = 0;  // slice 0, channel A
    const uint PWM_PIN_B = 1;  // slice 0, channel B
    gpio_set_function(PWM_PIN_A, GPIO_FUNC_PWM);
    gpio_set_function(PWM_PIN_B, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN_A);
    pwm_config config = pwm_get_default_config();
    // 125 MHz / 125 → 1 MHz counter; wrap = 999 → 1 kHz PWM
    pwm_config_set_clkdiv(&config, 125.0f);
    pwm_config_set_wrap(&config, 999);
    pwm_init(slice_num, &config, true);

    // --- Button pins (just examples, change to your wiring) ---
    const uint PIN_ACTIVE_BTN = 9;  // Start/enable button
    const uint PIN_LOGIC_BTN  = 10;  // Logic direction button
    const uint PIN_STOP_BTN   = 11;  // Emergency stop button

    gpio_init(PIN_ACTIVE_BTN);
    gpio_set_dir(PIN_ACTIVE_BTN, GPIO_IN);
    gpio_pull_down(PIN_ACTIVE_BTN);

    gpio_init(PIN_LOGIC_BTN);
    gpio_set_dir(PIN_LOGIC_BTN, GPIO_IN);
    gpio_pull_down(PIN_LOGIC_BTN);

    gpio_init(PIN_STOP_BTN);
    gpio_set_dir(PIN_STOP_BTN, GPIO_IN);
    gpio_pull_down(PIN_STOP_BTN);


    while (true) {
        pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
        pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
        sleep_ms(10);

        // uint16_t raw = adc_read();
        // printf("%u\n", raw);

        if (gpio_get(PIN_ACTIVE_BTN)) {
            sleep_ms(100);
            if (gpio_get(PIN_LOGIC_BTN)) {
            pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
            pwm_set_chan_level(slice_num, PWM_CHAN_B, 500);
            sleep_ms(500);

            }
            else {
                pwm_set_chan_level(slice_num, PWM_CHAN_A, 500);
                pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
                sleep_ms(200);
                while (!gpio_get(PIN_STOP_BTN) && adc_read() < 1500){
                    sleep_ms(10);
                }
                
            }
        
        }


    }

    return 0;
}
