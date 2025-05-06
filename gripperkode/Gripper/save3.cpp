#include "pico/stdlib.h"
#include "hardware/adc.h"
#include <cstdio>

int main() {
    // Initialize standard I/O for USB serial output.
    stdio_init_all();
    // Allow time for the USB serial connection to establish.
    sleep_ms(2000);

    // --- ADC Setup ---
    // Initialize the ADC hardware.
    adc_init();
    // Set up GPIO26 for analog input (ADC channel 0).
    adc_gpio_init(26);
    // Select ADC channel 0 for reading.
    adc_select_input(0);

    // --- Digital Output Setup on Pin 8 ---
    // Initialize GPIO8 as a digital output pin.
    gpio_init(8);
    gpio_set_dir(8, GPIO_OUT);
    gpio_put(8, 1);

    // Define the threshold for the ADC reading.
    // A value equal to or above this threshold will set GPIO8 high.
    const uint16_t threshold = 2048;  // Roughly half of the maximum ADC value (4095)

    while (true) {
        // Read the ADC value (range: 0 to 4095).
        uint16_t adc_val = adc_read();
        // Calculate the corresponding voltage (assuming 3.3V reference).
        float voltage = adc_val * (3.3f / 4095.0f);

        // Print the ADC raw value and the computed voltage.
        printf("ADC reading: %d, Voltage: %.2f V\n", adc_val, voltage);

        sleep_ms(500);
    }

    return 0;
}
// sudo minicom -D /dev/ttyACM0 -b 115200