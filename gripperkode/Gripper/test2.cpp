#include "pico/stdlib.h"
#include "hardware/pwm.h"

int main() {
    // Initialiserer standard I/O (bruges fx til debugging)
    stdio_init_all();

    // Definér PWM-pinnen
    const uint PWM_PIN1 = 0;
    const uint PWM_PIN2 = 1;

    // Sæt PWM_PIN til PWM-funktion
    gpio_set_function(PWM_PIN1, GPIO_FUNC_PWM);
    gpio_set_function(PWM_PIN2, GPIO_FUNC_PWM);

    // Hent PWM-slice nummeret for PWM_PIN
    uint slice_num1 = pwm_gpio_to_slice_num(PWM_PIN1);
    uint slice_num2 = pwm_gpio_to_slice_num(PWM_PIN2);
    // Konfigurer PWM:
    // Brug standard konfiguration og sæt wrap-værdien til 1000
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, 1000);
    pwm_init(slice_num1, &config, true);
    pwm_init(slice_num2, &config, true);



    // Hold programmet kørende
    while (true) {
        pwm_set_gpio_level(PWM_PIN1, 500);
        pwm_set_gpio_level(PWM_PIN2, 0);
        sleep_ms(5000);
        pwm_set_gpio_level(PWM_PIN1, 0);
        pwm_set_gpio_level(PWM_PIN2, 500);
        sleep_ms(5000);
    }

    return 0;
}
