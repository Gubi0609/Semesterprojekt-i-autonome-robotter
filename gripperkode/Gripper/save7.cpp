#include <cstdio>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

int main() {
    stdio_init_all();         // Aktiverer USB-CDC og (UART-stdio hvis slået til)

    adc_init();
    adc_gpio_init(27);        // ADC1 på GPIO27
    adc_select_input(1);

    const uint PIN_LED = 8;
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 1);     // Sæt GPIO8 høj

    // PWM pins
    const uint PWM_PIN_A = 0;  // slice 0, channel A
    const uint PWM_PIN_B = 1;  // slice 0, channel B

    // Configure both pins for PWM output
    gpio_set_function(PWM_PIN_A, GPIO_FUNC_PWM);
    gpio_set_function(PWM_PIN_B, GPIO_FUNC_PWM);

    // They’re on the same slice
    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN_A);

    // Get default slice configuration
    pwm_config config = pwm_get_default_config();

    // 125 MHz / 125 → 1 MHz counter; wrap = 999 → 1 kHz PWM
    pwm_config_set_clkdiv(&config, 125.0f);
    pwm_config_set_wrap(&config, 999);

    // Init & start PWM slice
    pwm_init(slice_num, &config, true);

    // 50 % duty = 500 / 1000 ticks
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 500);

    while (true) {
        uint16_t raw = adc_read();
        // Print rå værdi over USB
        printf("%u\n", raw);

        sleep_ms(500);
    }
    return 0;
}

// sudo minicom -D /dev/ttyACM0 -b 115200