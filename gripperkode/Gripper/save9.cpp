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
    pwm_config_set_clkdiv(&config, 125.0f);
    pwm_config_set_wrap(&config, 999);
    pwm_init(slice_num, &config, true);

   
    const uint BTN1 = 9;  
    const uint BTN2 = 12; 


    gpio_init(BTN1);
    gpio_set_dir(BTN1, GPIO_IN);
    gpio_pull_down(BTN1);

    gpio_init(BTN2);
    gpio_set_dir(BTN2, GPIO_IN);
    gpio_pull_down(BTN2);


    while (true) {
        pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
        pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
        sleep_ms(10);

        if (gpio_get(BTN1)) {
            // When BTN1 is pressed
            pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
            pwm_set_chan_level(slice_num, PWM_CHAN_B, 200);
            sleep_ms(700);
        }
        else if (gpio_get(BTN2)) {
            // When BTN2 is pressed
            pwm_set_chan_level(slice_num, PWM_CHAN_A, 200);
            pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
            sleep_ms(350);
            while (adc_read() < 920) {
                sleep_ms(5);
            }
        }
    }

    return 0;
}
// en måde at logge
// logge med tid
// far adc værdi til strøm
// skal være 24v
// andre målinger 