#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>

// Define pins
#define PWM_PIN1    0     // PWM output pin 1
#define PWM_PIN2    1     // PWM output pin 2
#define TRIG_PIN    8     // Trigger input pin (wait-for signal)
#define READ_PIN    9     // Pin to read once TRIG_PIN goes high
#define OUTPUT_PIN 10     // Additional digital output pin

// Volatile flag used by close_gripper (via interrupt)
volatile bool button_pressed = false;

// Interrupt callback for TRIG_PIN (used by close_gripper)
void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == TRIG_PIN) {
        button_pressed = true;
    }
}

// Initialize PWM on PWM_PIN1 and PWM_PIN2
void init_pwm() {
    gpio_set_function(PWM_PIN1, GPIO_FUNC_PWM);
    gpio_set_function(PWM_PIN2, GPIO_FUNC_PWM);
    uint slice1 = pwm_gpio_to_slice_num(PWM_PIN1);
    uint slice2 = pwm_gpio_to_slice_num(PWM_PIN2);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, 1000);  // 0–1000 resolution

    pwm_init(slice1, &config, true);
    pwm_init(slice2, &config, true);
    pwm_set_gpio_level(PWM_PIN1, 0);
    pwm_set_gpio_level(PWM_PIN2, 0);
    pwm_set_enabled(slice1, true);
    pwm_set_enabled(slice2, true);
}

// Initialize TRIG_PIN with pull-down (so it reads 0 when floating)
void init_button() {
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_IN);
    gpio_pull_down(TRIG_PIN);  // <<— use pull-down instead of pull-up
    gpio_set_irq_enabled_with_callback(
        TRIG_PIN,
        GPIO_IRQ_EDGE_RISE,
        true,
        &gpio_callback
    );
}

// Initialize OUTPUT_PIN as a digital output
void init_output_pin() {
    gpio_init(OUTPUT_PIN);
    gpio_set_dir(OUTPUT_PIN, GPIO_OUT);
    gpio_put(OUTPUT_PIN, 1);
}

// Initialize READ_PIN as an input
void init_read_pin() {
    gpio_init(READ_PIN);
    gpio_set_dir(READ_PIN, GPIO_IN);
    // If you need a default on READ_PIN, you can also:
    // gpio_pull_up(READ_PIN);
    // or
    // gpio_pull_down(READ_PIN);
}

// Close the gripper: ramp PWM_PIN1 until TRIG_PIN interrupt fires
void close_gripper() {
    button_pressed = false;
    pwm_set_gpio_level(PWM_PIN1, 300);
    while (!button_pressed) {
        sleep_ms(10);
    }
    pwm_set_gpio_level(PWM_PIN1, 0);
}

// Open the gripper for a fixed time
void open_gripper() {
    pwm_set_gpio_level(PWM_PIN2, 300);
    sleep_ms(400);
    pwm_set_gpio_level(PWM_PIN2, 0);
}

// Wait until TRIG_PIN goes high, then read and return READ_PIN (0 or 1)
int waitforpico() {
    while (gpio_get(TRIG_PIN) == 0) {
        sleep_ms(1);
    }
    return gpio_get(READ_PIN);
}

int main() {
    stdio_init_all();

    // Initialize peripherals
    init_output_pin();
    init_pwm();
    init_button();
    init_read_pin();

    // Main loop: wait on TRIG_PIN, then act on READ_PIN value
    while (true) {
        int signal = waitforpico();
        if (signal == 1) {
            close_gripper();
        } else {
            open_gripper();
        }
        sleep_ms(50);  // debounce/delay between cycles
    }

    return 0;
}
