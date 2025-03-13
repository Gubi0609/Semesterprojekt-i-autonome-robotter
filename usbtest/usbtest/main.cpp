#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>

int main() {
    // Initialize USB serial (CDC) communication.
    stdio_init_all();

    // Wait until the host (PC) connects to the Pico via USB.
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    
    // Initialize the external LED pin on GP5.
    const uint LED_PIN = 5;  // Using GP5 for the external LED.
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    char buffer[64];
    while (true) {
        // Prompt for input
        printf("Enter command (on/off):\n");

        // Read a line of input from the USB serial port.
        int i = 0;
        memset(buffer, 0, sizeof(buffer));
        int ch;
        while ((ch = getchar_timeout_us(1000000)) != PICO_ERROR_TIMEOUT && ch != '\n') {
            if (i < sizeof(buffer) - 1) {
                buffer[i++] = (char) ch;
            }
        }
        buffer[i] = '\0';  // Null-terminate the string

        // Process input: turn LED on or off based on command.
        if (strcmp(buffer, "on") == 0) {
            gpio_put(LED_PIN, 1);
            printf("LED on GP5 turned on.\n");
        } else if (strcmp(buffer, "off") == 0) {
            gpio_put(LED_PIN, 0);
            printf("LED on GP5 turned off.\n");
        } else {
            printf("Unknown command: %s\n", buffer);
        }
    }
    
    return 0;
}
