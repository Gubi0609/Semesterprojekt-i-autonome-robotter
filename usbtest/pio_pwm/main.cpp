/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

 #include <iostream>
 #include "pico/stdlib.h"
 #include "hardware/pio.h"
 #include "pwm.pio.h"
 
 // Skriv 'period' til input shift register
 void pio_pwm_set_period(PIO pio, uint sm, uint32_t period) {
     pio_sm_set_enabled(pio, sm, false);
     pio_sm_put_blocking(pio, sm, period);
     pio_sm_exec(pio, sm, pio_encode_pull(false, false));
     pio_sm_exec(pio, sm, pio_encode_out(pio_isr, 32));
     pio_sm_set_enabled(pio, sm, true);
 }
 
 // Skriv 'level' til TX FIFO. State machine vil kopiere dette til X.
 void pio_pwm_set_level(PIO pio, uint sm, uint32_t level) {
     pio_sm_put_blocking(pio, sm, level);
 }
 
 int main() {
     stdio_init_all();
 
 #ifdef PICO_DEFAULT_LED_PIN
     // Brug PIO 0 og vælg state machine 0 (evt. tilpas, hvis du har flere ledige)
     PIO pio = pio0;
     int sm = 0;
     
     // Indlæs PWM-programmet til PIO, og få offset for programmet
     uint offset = pio_add_program(pio, &pwm_program);
     std::cout << "Loaded program at " << offset << std::endl;
     
     // Initialiser PWM-programmet: Tilslut programmet til den ønskede pin
     pwm_program_init(pio, sm, offset, PICO_DEFAULT_LED_PIN);
     
     // Sæt period til maks (wrap-værdi 2^16 - 1)
     pio_pwm_set_period(pio, sm, (1u << 16) - 1);
 
     int level = 0;
     while (true) {
         std::cout << "Level = " << level << std::endl;
         // Næsten kvadreret kurve for at få en glidende effekt
         pio_pwm_set_level(pio, sm, level * level);
         level = (level + 1) % 256;
         sleep_ms(10);
     }
 #else
 #warning pio/pwm example requires a board with a regular LED
     std::cerr << "Default LED pin was not defined" << std::endl;
 #endif
 
     return 0;
 }
 