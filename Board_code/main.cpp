#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <array>
#include <sstream>

// Structure representing a move with source and destination coordinates.
// Files: 'a'-'h' -> 0-7, Ranks: '1'-'8' -> 0-7.
struct Move {
    std::pair<int, int> source;       // e.g., a2 -> (0, 1)
    std::pair<int, int> destination;  // e.g., a4 -> (0, 3)
};

// Converts a move string (e.g., "a2a4") into a Move struct.
Move convertMove(const std::string &moveStr) {
    if (moveStr.size() != 4) {
        return {{-1, -1}, {-1, -1}}; // Invalid move if not 4 characters.
    }
    int srcFile = moveStr[0] - 'a'; // 'a' becomes 0, 'b' becomes 1, etc.
    int srcRank = moveStr[1] - '1'; // '1' becomes 0, '2' becomes 1, etc.
    int dstFile = moveStr[2] - 'a';
    int dstRank = moveStr[3] - '1';
    return {{srcFile, srcRank}, {dstFile, dstRank}};
}

// Parses a single input string containing moves separated by spaces
// and returns a vector of Move structs.
std::vector<Move> parseMoves(const std::string &input) {
    std::vector<Move> moves;
    std::istringstream iss(input);
    std::string token;
    while (iss >> token) {
        if (token.size() == 4) {
            moves.push_back(convertMove(token));
        }
    }
    return moves;
}

// Returns a vector of moves that originate from the given square.
// The square is specified by its file and rank (0-indexed).
std::vector<Move> getLegalMovesForSquare(const std::vector<Move>& moves, int file, int rank) {
    std::vector<Move> result;
    for (const auto &move : moves) {
        if (move.source.first == file && move.source.second == rank) {
            result.push_back(move);
        }
    }
    return result;
}

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

bool wait = true;

std::vector<Move> moves;

State state = WAITING_FOR_FIRST;

// Definer hvilke pins, der bruges til rækker (outputs) og kolonner (inputs)
// Juster disse pin-numre til din hardwareopsætning
const uint rowPins[numRows] = {0, 1, 2, 3, 19, 5, 6, 7};
const uint colPins[numCols] = {8, 9, 10, 11, 12, 13, 14, 15};
const uint colPins_light[numCols] = {16, 17, 18, 4, 20, 21, 22, 23};

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
    for(int j = 0; j < 16; j++) {
        gpio_put(colPins_light[j], 1);    
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
    
    // I starten af main-loopen venter vi på en vector-string fra PC'en.
    // Denne del blokerer indtil der modtages en linje (afsluttet med newline)
    while (wait == true) {
        char inputBuffer[256];
        int index = 0;
        printf("Vent på vector string fra PC...\n");
        while (true) {
            int ch = getchar();  // Blokerende læsning
            if (ch == '\n') break;
            if (ch != EOF) {
                if (index < (int)sizeof(inputBuffer) - 1) {
                    inputBuffer[index++] = (char) ch;
                }
            }
        }
        inputBuffer[index] = '\0';  // Null-terminér strengen
        printf("Modtaget vector string: %s\n", inputBuffer);
        
        // Hvis en ikke-tom string er modtaget, afslut venteløkken
        if (strlen(inputBuffer) > 0) {
            wait = false;
            moves = parseMoves(inputBuffer); //moves fra stock blvier laver om til tal til matrix
            // Print de konverterede træk til den serielle konsol
            printf("Parserede træk:\n");
            for (const auto &move : moves) {
                printf("Fra (%d, %d) -> Til (%d, %d)\n", 
                    move.source.first, move.source.second, 
                    move.destination.first, move.destination.second);
                }
    }   sleep_us(100000);
    
    
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

                // Hvis der sker en ændring i input (evt. med debouncing)
                if (matrix[row][col].current != matrix[row][col].last) {
                    if (state == INITIALIZE) {
                        state = WAITING_FOR_FIRST;
                    }
                    else if (state == WAITING_FOR_FIRST) {
                        // Første tryk registreret: gem koordinater
                        first_col = matrix[row][col].col;
                        first_row = matrix[row][col].row;

                        // Find alle lovlige træk for det valgte felt
                        std::vector<Move> legalMoves = getLegalMovesForSquare(moves, first_row, first_col);
                        // Print out the legal moves from the given square.
                        printf("Legal moves for square (%d, %d):\n", first_row, first_col);
                        for (const auto &move : legalMoves) {
                            matrix[move.destination.first][move.destination.second].lys = true; // Tænd LED for lovlige træk
                            // We print only the destination coordinate.
                            printf("-> Destination: (%d, %d)\n", move.destination.first, move.destination.second);
    }
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
                            state = INITIALIZE;
                            // Send trækket til PC'en
                            // vent på svar fra pc
                            //evt send vidre til gripper
                            //opdater legel moves fra stockfish
                        }
                    }
             
                }
            
            sleep_us(100);
            }
        
        }
        
        
    }
    
    return 0;
}
}