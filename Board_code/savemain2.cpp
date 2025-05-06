#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <array>
#include <sstream>
#include <utility>

// Structure representing a move with source and destination coordinates.
// Files: 'a'-'h' -> 0-7, Ranks: '1'-'8' -> 0-7.
struct Move {
    std::pair<int, int> source;       // (file, rank)
    std::pair<int, int> destination;  // (file, rank)
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

// Function to convert a Move struct back into a chess move string.
std::string moveToString(const Move &move) {
    std::string moveStr;
    moveStr.push_back('a' + move.source.first);
    moveStr.push_back('1' + move.source.second);
    moveStr.push_back('a' + move.destination.first);
    moveStr.push_back('1' + move.destination.second);
    return moveStr;
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



// Structure for each point in the matrix
struct Point {
    int current;       // current value (0 or 1)
    int last;          // previous value
    int row;           // row position (rank)
    int col;           // column position (file)
    bool lys;          // LED status (on/off)
};

const int numRows = 8;
const int numCols = 8;

int first_col = 0;
int first_row = 0;
int second_col = 0;
int second_row = 0;

bool wait = true;
int runs = 0;
std::vector<Move> moves;

int state = 1;

// Define pins for rows (outputs) and columns (inputs)
// Adjust these pin numbers to your hardware setup.
const uint rowPins[numRows] = {0, 1, 2, 3, 4, 5, 6, 7};
const uint colPins[numCols] = {8, 9, 10, 11, 12, 13, 14, 15};
const uint colPins_light[numCols] = {16, 17, 18, 19, 20, 21, 22, 23};

// Function to initialize all pins
void initPins() {
    // Initialize row pins as outputs and set them low.
    for (int i = 0; i < numRows; i++) {
        gpio_init(rowPins[i]);
        gpio_set_dir(rowPins[i], GPIO_OUT);
        gpio_put(rowPins[i], 0);
    }
    
    // Initialize column pins as inputs with pull-down resistors.
    for (int j = 0; j < numCols; j++) {
        gpio_init(colPins[j]);
        gpio_set_dir(colPins[j], GPIO_IN);
        gpio_pull_down(colPins[j]);
    }
    
    // Initialize light pins as outputs and set them high.
    for (int i = 0; i < numCols; i++) {
        gpio_init(colPins_light[i]);
        gpio_set_dir(colPins_light[i], GPIO_OUT);
        gpio_put(colPins_light[i], 1);
    }
}

// Function to turn off all lights in the matrix.
void clearLights(Point matrix[numRows][numCols]) {
    for (int i = 0; i < numRows; i++) {
        for (int j = 0; j < numCols; j++) {
            matrix[i][j].lys = false;
        }
    }
   
    for (int j = 0; j < numCols; j++) {
        gpio_put(colPins_light[j], 1);    
    }
}

// Function that waits for input from the PC, parses it into a vector of Moves, and returns it.
std::vector<Move> waitForMovesFromPC() {
    char inputBuffer[256];
    while (true) {
        int index = 0;
        // printf("Vent på vector string fra PC...\n");
        
        // Blocking read until newline.
        while (true) {
            int ch = getchar();
            if (ch == '\n') break;
            if (ch != EOF) {
                if (index < (int)sizeof(inputBuffer) - 1) {
                    inputBuffer[index++] = (char)ch;
                }
            }
        }
        inputBuffer[index] = '\0';  // Null-terminate the string.
        
        // If input is not empty, parse and return moves.
        if (strlen(inputBuffer) > 0) {
            std::vector<Move> moves = parseMoves(inputBuffer);
            // printf("Parserede træk:\n");
            //for (const auto &move : moves) {
                // printf("Fra (%d, %d) -> Til (%d, %d)\n", 
                    // move.source.first, move.source.second, 
                   //  move.destination.first, move.destination.second);
            }
            return moves;
        }
        sleep_us(500);  // Short delay before trying again.
    }


int main() {

    stdio_init_all();  // Initialize USB CDC.
    initPins();
    
    // Create and initialize an 8x8 matrix of Point objects.
    Point matrix[numRows][numCols];
    for (int i = 0; i < numRows; i++) {
        for (int j = 0; j < numCols; j++) {
            matrix[i][j].row = i;  // rank
            matrix[i][j].col = j;  // file
            matrix[i][j].current = 0;
            matrix[i][j].last = 0;
            matrix[i][j].lys = false;
        }
    }
    
    moves = waitForMovesFromPC();
    
    while (true) {
        
        // Scan the matrix: set one row HIGH at a time.
        for (int row = 0; row < numRows; row++) {
            // Set all rows LOW first.
            for (int r = 0; r < numRows; r++) {
                gpio_put(rowPins[r], 0);
            }
            // Set the active row HIGH.
            gpio_put(rowPins[row], 1);
            
            // Short delay for signal stabilization.
            sleep_us(1000);
            
            // Read column inputs for the active row and update each point.
            for (int col = 0; col < numCols; col++) {
                runs++;
                // Control LED based on the matrix point's light status.
                if (matrix[row][col].lys == true) {
                    gpio_put(colPins_light[col], 0);
                } else {
                    gpio_put(colPins_light[col], 1);
                }
                // Save the previous value.
                matrix[row][col].last = matrix[row][col].current;
                // Read the new value.
                matrix[row][col].current = gpio_get(colPins[col]);
                // printf("row: %d, col: %d, current: %d, last: %d\n", row, col, matrix[row][col].current, matrix[row][col].last);

                // If there's a change in input
                if (runs < 65) {
                    // printf("runs: %d\n", runs);
                    
                }
                else if (matrix[row][col].current != matrix[row][col].last) {
                    sleep_us(1000);  // Debounce delay.
                    
                    if (state == 1) {
                        
                        first_col = matrix[row][col].col;
                        first_row = matrix[row][col].row;
                        // printf("runs: %d\n", runs);
                        
                        std::vector<Move> legalMoves = getLegalMovesForSquare(moves, first_col, first_row);
                        // printf("Legal moves for square (file: %d, rank: %d):\n", first_col, first_row);
                        for (const auto &move : legalMoves) {
                            // Activate LED for legal move destination.
                            matrix[move.destination.second][move.destination.first].lys = true;
                            // printf("-> Destination (file: %d, rank: %d)\n", move.destination.first, move.destination.second);
                        }
                        state = 2;
                    }
                    else if (state == 2) {
                        // Second press detected.
                        second_col = matrix[row][col].col;
                        second_row = matrix[row][col].row;
                        clearLights(matrix);
                        // printf("2. brik sat \n");
                        
                        if (first_col == second_col && first_row == second_row) {
                            state = 1;
                        }
                        else {
                            state = 1;
                            runs = 0;
                            
                            Move movemade = {{first_col, first_row}, {second_col, second_row}};
                            std::string movetosend = moveToString(movemade);
                            // printf("sender move \n");
                            printf("%s", movetosend.c_str());
                            moves = waitForMovesFromPC();
                            
                            //andet logik efter træk her
                        }
                    }
                }
                sleep_us(100);
            }
        }
    }
    
    return 0;
}
// sudo minicom -D /dev/ttyACM0 -b 115200