#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>

// Definér tydelige tilstande
enum State {
    INITIALIZE,
    WAITING_FOR_FIRST,
    WAITING_FOR_SECOND,
    WAITING_FOR_RESPONSE
};

// Struktur for hvert punkt i matrixen
struct Point {
    bool current;       // Nuværende værdi (0 eller 1)
    bool last;          // Forrige værdi
    int row;            // Række-position
    int col;            // Kolonne-position
    bool lys;           // LED-status (tændt/slukket)
};

const int numRows = 8;
const int numCols = 8;

int first_col = 0;
int first_row = 0;
int second_col = 0;
int second_row = 0;

State state = WAITING_FOR_FIRST;

// Definer hvilke pins, der bruges til rækker (outputs) og kolonner (inputs)
// Juster disse pin-numre til din hardwareopsætning
const uint rowPins[numRows] = {0, 1, 2, 3, 4, 5, 6, 7};
const uint colPins[numCols] = {8, 9, 10, 11, 12, 13, 14, 15};
const uint colPins_light[numCols] = {16, 17, 18, 19, 20, 21, 22, 23};

// Funktion til at initialisere alle pins
void initPins() {
    // Initialiser række-pins som outputs og sæt dem til lav
    for (int i = 0; i < numRows; i++) {
        gpio_init(rowPins[i]);
        gpio_set_dir(rowPins[i], GPIO_OUT);
        gpio_put(rowPins[i], 0);
    }
    
    // Initialiser kolonne-pins som inputs med pull-down-modstand
    for (int j = 0; j < numCols; j++) {
        gpio_init(colPins[j]);
        gpio_set_dir(colPins[j], GPIO_IN);
        gpio_pull_down(colPins[j]);
    }
    
    // Initialiser light-pins som outputs og sæt dem til high
    for (int i = 0; i < numCols; i++) {
        gpio_init(colPins_light[i]);
        gpio_set_dir(colPins_light[i], GPIO_OUT);
        gpio_put(colPins_light[i], 1);
    }
}

// Funktion til at slukke alle lys i matrixen
void clearLights(Point matrix[numRows][numCols]) {
    for (int i = 0; i < numRows; i++) {
        for (int j = 0; j < numCols; j++) {
            matrix[i][j].lys = false;
        }
    }
}

int main() {
    stdio_init_all();  // Initialiser USB CDC
    initPins();
    
    // Opret og initialiser en 8x8 matrix af Point-objekter
    Point matrix[numRows][numCols];
    for (int i = 0; i < numRows; i++) {
        for (int j = 0; j < numCols; j++) {
            matrix[i][j].row = i;
            matrix[i][j].col = j;
            matrix[i][j].current = false;
            matrix[i][j].last = false;
            matrix[i][j].lys = false;
        }
    }
    
    while (true) {
        // Scan matrixen: sæt én række ad gangen til HIGH
        for (int row = 0; row < numRows; row++) {
            // Sørg for, at alle rækker sættes til lav først
            for (int r = 0; r < numRows; r++) {
                gpio_put(rowPins[r], 0);
            }
            // Sæt den aktive række til HIGH
            gpio_put(rowPins[row], 1);
            
            // Kort delay for at signalet kan stabilisere sig
            sleep_us(100);
            
            // Læs kolonne-inputs for den aktive række og opdater hvert point
            for (int col = 0; col < numCols; col++) {
                // Styr LED baseret på matrixpunktets lys-status
                if (matrix[row][col].lys == true) {
                    gpio_put(colPins_light[col], 0);
                } else {
                    gpio_put(colPins_light[col], 1);
                }
                // Gem forrige værdi
                matrix[row][col].last = matrix[row][col].current;
                // Læs den nye værdi
                matrix[row][col].current = gpio_get(colPins[col]);

                // Hvis der sker en ændring i input (evt. med tilføjet debouncing)
                if (matrix[row][col].current != matrix[row][col].last) {
                    if (state == INITIALIZE) {
                        state = WAITING_FOR_FIRST;
                    }
                    else if (state == WAITING_FOR_FIRST) {
                        // Første tryk registreret: gem koordinater
                        first_col = matrix[row][col].col;
                        first_row = matrix[row][col].row;
                        
                        // SEND MOVES TIL MAIN: send koordinater via USB til PC
                        printf("MOVE:%d,%d\n", first_row, first_col);
                        
                        // Vent på svar fra PC (samler tegn indtil newline eller timeout)
                        char response[64];
                        int i = 0;
                        while (true) {
                            int ch = getchar_timeout_us(1000000); // 1 sekund timeout per tegn
                            if (ch == PICO_ERROR_TIMEOUT) {
                                // Timeout: stop og skriv fejlbesked
                                printf("Ingen respons modtaget fra PC.\n");
                                break;
                            }
                            if (ch == '\n') break;
                            if (i < (int)sizeof(response) - 1) {
                                response[i++] = (char) ch;
                            }
                        }
                        response[i] = '\0';
                        printf("Modtog respons: %s\n", response);
                        
                        // Efter at have modtaget respons, gå videre til næste tilstand
                        state = WAITING_FOR_SECOND;
                    }
                    else if (state == WAITING_FOR_SECOND) {
                        // Andet tryk registreret
                        second_col = matrix[row][col].col;
                        second_row = matrix[row][col].row;
                        clearLights(matrix);
                        
                        // Hvis det samme felt er trykket, annulleres handlingen
                        if (first_col == second_col && first_row == second_row) {
                            state = WAITING_FOR_FIRST;
                        }
                        else {
                            state = WAITING_FOR_RESPONSE;
                            // Her kan fx en yderligere kommunikation implementeres
                            // fx: send begge træk til PC for at få en samlet respons
                        }
                    }
                }
            }
        }
        
        // Udskriv matrixens tilstand (for debugging via USB serial)
        //printf("Matrix state:\n");
        //for (int i = 0; i < numRows; i++) {
        //    for (int j = 0; j < numCols; j++) {
        //        printf("(%d,%d): cur=%d, last=%d  ", 
        //            matrix[i][j].row, matrix[i][j].col, 
        //            matrix[i][j].current ? 1 : 0, 
        //            matrix[i][j].last ? 1 : 0);
        //    }
        //    printf("\n");
        //}
        //printf("\n");
        //
        //sleep_ms(500);  // Kort pause mellem scanninger
    }
    
    return 0;
}
