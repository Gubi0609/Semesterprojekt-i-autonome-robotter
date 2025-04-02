#include "pico/stdlib.h"
#include "hardware/pwm.h"

// Definér pins: PWM output på GPIO 1 og knap på GPIO 15
#define PWM_PIN 1
#define BUTTON_PIN 15

// En volatile flag, som interrupt handleren sætter, når knappen trykkes
volatile bool button_pressed = false;

// Interrupt callback for knappen
void gpio_callback(uint gpio, uint32_t events) {
    // Sæt flaget, hvis interrupt kommer fra knappen (GPIO 15)
    if (gpio == BUTTON_PIN) {
        button_pressed = true;
    }
}

int main() {
    stdio_init_all();

    // Konfigurer PWM-pin (GPIO 1)
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, 1000);  // Wrap-værdi: 0-1000
    pwm_init(slice_num, &config, true);
    pwm_set_gpio_level(PWM_PIN, 500);     // Sæt duty cycle til 50% (500 ud af 1000)
    pwm_set_enabled(slice_num, true);

    // Konfigurer knappen (GPIO 15) som input med pull-up
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);

    // Sæt interrupt på knappen: udløses ved faldende flank (knappen trykkes ned)
    gpio_set_irq_enabled_with_callback(BUTTON_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    // Hovedløkken: Indtil knappen trykkes, hold PWM duty cycle på 500
    while (!button_pressed) {
        sleep_ms(10);
    }

    // Når knappen er trykket, ændres PWM-signalet. Her sætter vi duty cycle til 0.
    pwm_set_gpio_level(PWM_PIN, 0);

    // Eventuelt: Lad koden blive her, så PWM forbliver med duty cycle 0
    while (true) {
        tight_loop_contents();
    }

    return 0;
}
