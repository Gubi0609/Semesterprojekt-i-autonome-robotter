#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <string>
#include <cstring>

// Define pins
#define PWM_PIN1 0       // PWM output pin
#define PWM_PIN2 1       // PWM output pin
#define BUTTON_PIN 28    // Button input pin
#define OUTPUT_PIN 8     // Additional digital output pin

// Volatile flag set by the button interrupt
volatile bool button_pressed = false;

// Interrupt callback for the button
void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == BUTTON_PIN) {
        button_pressed = true;
    }
}

// Initialize PWM on PWM_PIN
void init_pwm() {
    gpio_set_function(PWM_PIN1, GPIO_FUNC_PWM);
    gpio_set_function(PWM_PIN2, GPIO_FUNC_PWM);
    uint slice_num1 = pwm_gpio_to_slice_num(PWM_PIN1);
    uint slice_num2 = pwm_gpio_to_slice_num(PWM_PIN2);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, 1000);  // Resolution from 0 to 1000
    pwm_init(slice_num1, &config, true);
    pwm_init(slice_num2, &config, true);
    // Set initial duty cycle to 0
    pwm_set_gpio_level(PWM_PIN1, 0);
    pwm_set_gpio_level(PWM_PIN2, 0);
    pwm_set_enabled(slice_num1, true);
    pwm_set_enabled(slice_num2, true);
}

// Initialize the button on BUTTON_PIN with a pull-up and interrupt
void init_button() {
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);
    gpio_set_irq_enabled_with_callback(BUTTON_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
}

// Initialize an additional digital output (e.g., OUTPUT_PIN)
void init_output_pin() {
    gpio_init(OUTPUT_PIN);
    gpio_set_dir(OUTPUT_PIN, GPIO_OUT);
    gpio_put(OUTPUT_PIN, 1);
}

// Function to "close the gripper" when the button is pressed.
// It sets the PWM duty cycle to 500 and waits until the button is pressed,
// then sets the duty cycle to 0.
void close_gripper() {
    pwm_set_gpio_level(PWM_PIN1, 500);
    while (!button_pressed) {
        sleep_ms(10);
    }
    // Set PWM duty cycle to 0 to "close" the gripper.
    pwm_set_gpio_level(PWM_PIN1, 0);
}

// Function to "open the gripper" for demonstration purposes.
void open_gripper() {
    pwm_set_gpio_level(PWM_PIN2, 500);
    sleep_ms(5000);
    pwm_set_gpio_level(PWM_PIN2, 0);
}

// Function to wait for a command from the PC and return 1 or 0 as an int.
// It expects to receive only a '1' or '0' character.
int waitforpc() {
    while (true) {
        int ch = getchar();
        if (ch == '1' || ch == '0') {
            return (ch == '1') ? 1 : 0;
        }
        // Ignore any other characters.
    }
}

int main() {
    stdio_init_all();

    // Initialize peripherals
    init_output_pin();
    init_pwm();
    init_button();

    // Main loop: wait for a '1' or '0' from the PC and act accordingly.
    while (true) {
        int command = waitforpc();
        // For example, "1" means close the gripper and "0" means open it.
        if (command == 1) {
            close_gripper();
        } else if (command == 0) {
            open_gripper();
        }
    }

    return 0;
}
