#include "pico/stdlib.h"
#include <stdio.h>
#include <string>

// Struktur for hvert punkt i matrixen
struct Point {
    bool current;       // Nuværende værdi (0 eller 1)
    bool last;          // Forrige værdi
    int row;            // Række-position
    int col;            // Kolonne-position
    std::string feature; // Ekstra egenskab (kan tilpasses)
};

const int numRows = 8;
const int numCols = 8;

int first_col = 0;
int first_row = 0;

int second_col = 0;
int second_row = 0;

int state = 0;

// Definer hvilke pins, der bruges til rækker (outputs) og kolonner (inputs)
// Juster disse pin-numre til din hardwareopsætning
const uint rowPins[numRows] = {0, 1, 2, 3, 4, 5, 6, 7};
const uint colPins[numCols] = {8, 9, 10, 11, 12, 13, 14, 15};
const uint colPins_light[numCols] = {16, 17, 18, 19, 20, 21, 22, 23};

int main() {
    stdio_init_all();

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
        for (int i = 0; i < numRows; i++) {
            gpio_init(colPins_light[i]);
            gpio_set_dir(colPins_light[i], GPIO_OUT);
            gpio_put(colPins_light[i], 1);
        }
    
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
                // indstil lys
                if (matrix[row][col].lys == true) {
                    gpio_put(colPins[col], 0);
                } else {
                    gpio_put(colPins[col], 1);
                }
                // Gem forrige værdi
                matrix[row][col].last = matrix[row][col].current;
                // Læs den nye værdi
                matrix[row][col].current = gpio_get(colPins[col]);

                if (matrix[row][col].current != matrix[row][col].last) {
                    if (state == 0) {
                        // snak med stockfish og find alle moves
                        // sæt lys til true beseret på input
                        first_col = matrix[row][col].col;
                        first_row = matrix[row][col].row;
                        //sæt første state
                        state = 1;
                        }
                    if (state == 1) {
                        second_col = matrix[row][col].col;
                        second_row = matrix[row][col].row;
                        //sæt anden state
                        state = 2;
                        //sluk alt lys
                        for (int i = 0; i < numRows; i++) {
                            for (int j = 0; j < numCols; j++) {
                                matrix[i][j].lys = false;
                            }
                        }
                        if (first_col == second_col && first_row == second_row) {
                            //gå tilbage til state 0
                            state = 0;
                        
                        }
                        else {
                            //send move til stockfish
                            while (state == 2) {
                                //vent på svar fra main
                                //når svar er modtaget gå tilbage til state 0
                            }
                        }
                        
                    }

                }
            }
        }
        
        // Udskriv matrixens tilstand med alle variabler
        printf("Matrix state:\n");
        for (int i = 0; i < numRows; i++) {
            for (int j = 0; j < numCols; j++) {
                printf("(%d,%d): cur=%d, last=%d, feature=%s  ", 
                    matrix[i][j].row, matrix[i][j].col, 
                    matrix[i][j].current ? 1 : 0, 
                    matrix[i][j].last ? 1 : 0, 
                    matrix[i][j].feature.c_str());
            }
            printf("\n");
        }
        printf("\n");
        
        // Vent et stykke mellem scanninger
        sleep_ms(500);
    }
    
    return 0;
}
