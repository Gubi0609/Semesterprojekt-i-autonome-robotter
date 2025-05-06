#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include <cstdio> 

#define PWM_PIN1 0
#define PWM_PIN2 1
#define OUTPUT_PIN 8
#define ADC_PIN 26

void init_pwm() {
    gpio_set_function(PWM_PIN1, GPIO_FUNC_PWM);
    gpio_set_function(PWM_PIN2, GPIO_FUNC_PWM);
    uint slice1 = pwm_gpio_to_slice_num(PWM_PIN1);
    uint slice2 = pwm_gpio_to_slice_num(PWM_PIN2);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, 1000);
    pwm_init(slice1, &config, true);
    pwm_init(slice2, &config, true);
    pwm_set_gpio_level(PWM_PIN1, 0);
    pwm_set_gpio_level(PWM_PIN2, 0);
    pwm_set_enabled(slice1, true);
    pwm_set_enabled(slice2, true);
}

void init_adc() {
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(0);
}

void init_output() {
    gpio_init(OUTPUT_PIN);
    gpio_set_dir(OUTPUT_PIN, GPIO_OUT);
    gpio_put(OUTPUT_PIN, 1);
}

void close_gripper() {
    pwm_set_gpio_level(PWM_PIN1, 300);
    sleep_ms(100);
    while (true) {
        uint16_t res = adc_read();
        float voltage = res * (3.3f / 4095.0f);
        if (voltage > 1.0f) break;
        sleep_ms(1);
    }
    pwm_set_gpio_level(PWM_PIN1, 0);
}

void open_gripper() {
    pwm_set_gpio_level(PWM_PIN2, 300);
    sleep_ms(400);
    pwm_set_gpio_level(PWM_PIN2, 0);
}

int waitforpc() {
    while (true) {
        int ch = getchar();
        if (ch == '1' || ch == '0')
            return (ch == '1') ? 1 : 0;
    }
}

int main() {
    stdio_init_all();
    init_output();
    init_pwm();
    init_adc();
    while (true) {
        int cmd = waitforpc();
        if (cmd == 1)
            close_gripper();
        else if (cmd == 0)
            open_gripper();
    }
    return 0;
}
