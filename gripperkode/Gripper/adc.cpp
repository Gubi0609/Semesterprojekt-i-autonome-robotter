#include <cstdio>
#include "pico/stdlib.h"
#include "hardware/adc.h"

int main() {
    stdio_init_all();         // Aktiverer USB-CDC og (UART-stdio hvis slået til)

    adc_init();
    adc_gpio_init(27);        // ADC1 på GPIO27
    adc_select_input(1);

    const uint PIN_LED = 8;
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 1);     // Sæt GPIO8 høj

    while (true) {
        uint16_t raw = adc_read();
        // Print rå værdi over USB
        printf("%u\n", raw);

        sleep_ms(500);
    }
    return 0;
}

// sudo minicom -D /dev/ttyACM0 -b 115200