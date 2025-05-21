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
const uint rowPins[numRows] = {16, 15, 14, 13, 12, 11, 10, 1};
const uint colPins[numCols] = {27, 22, 20, 18, 8, 6, 4, 2};
const uint colPins_light[numCols] = {28, 26, 21, 19, 9, 7, 5, 3};

// Function to initialize all pins
void initPins() {
    gpio_init(0);
    gpio_set_dir(0, GPIO_OUT);
    gpio_put(0, 0);
    gpio_init(17);
    gpio_set_dir(17, GPIO_OUT);
    gpio_put(17, 0);

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

std::vector<Move> waitForMovesFromPC() {
    sleep_ms(50);
    for (int r = 0; r < numRows; r++) {
        gpio_put(rowPins[r], 0);
    }
    gpio_put(rowPins[6], 1);
    for (int j = 0; j < numCols; j++) {
        gpio_put(colPins_light[j], 0);    
    }

    char inputBuffer[512];
    while (true) {
        int index = 0;
        // block until newline…
        while (true) {
            int ch = getchar();
            if (ch == '\n' || ch == EOF) break;
            if (index < sizeof(inputBuffer)-1) {
                inputBuffer[index++] = (char)ch;
            }
        }
        inputBuffer[index] = '\0';
        if (index > 0) {
            for (int i = 6; i >= 1; i--) {
                gpio_put(rowPins[i], 0);   
                gpio_put(rowPins[i-1], 1); 
                sleep_ms(50);
            }
            gpio_put(rowPins[0], 0);
            gpio_put(rowPins[6], 0);
            for (int j = 0; j < numCols; j++) {
                gpio_put(colPins_light[j], 1);    
            }
            // parse and return immediately
            sleep_ms(50);
            return parseMoves(inputBuffer);
        }
        sleep_us(500);
    }
}
void gripper() {

    sleep_ms(5);
    for (int r = 0; r < numRows; r++) {
        gpio_put(rowPins[r], 0);
    }
    gpio_put(rowPins[7], 1);
    for (int j = 0; j < numCols; j++) {
        gpio_put(colPins_light[j], 0);    
    }

    while (true){
        int ch = getchar();
        if (ch == '0') {
            gpio_put(0, 1);
            sleep_ms(50);
            gpio_put(0, 0);
        }
        else if (ch == '1'){
            gpio_put(17, 1);
            sleep_ms(50);
            gpio_put(17, 0);
        }
        else if (ch == '2'){
            break;
        }
        sleep_ms(1);
    }

    gpio_put(rowPins[7], 0);
    for (int j = 0; j < numCols; j++) {
        gpio_put(colPins_light[j], 1);    
    }
}

// Scans both row 0 and 1 until both have 8 magnets placed
void waitForInitialMagnets(Point matrix[numRows][numCols]) {
    bool done[2] = {false, false};
    int setup = 0;

    while (!(done[0] && done[1])) {

        // Scan row 0 and row 1
        for (int row = 0; row < 2; ++row) {
            // Ensure all light pins are HIGH (LEDs off) before activating a row
            for (int j = 0; j < numCols; j++) {
                gpio_put(colPins_light[j], 1);
            }

            // Set all rows LOW first.
            for (int r = 0; r < numRows; r++) {
                gpio_put(rowPins[r], 0);
            }
            
            // Set the active row HIGH.
            gpio_put(rowPins[row], 1);
            
            int count = 0;
            sleep_us(100);
            for (int col = 0; col < numCols; ++col) {
                setup++;
                // Control LED based on the matrix point's light status.
                if (matrix[row][col].lys == true) {
                    gpio_put(colPins_light[col], 0);
                } else {
                    gpio_put(colPins_light[col], 1);
                }

                sleep_us(200); 

                matrix[row][col].current = gpio_get(colPins[col]);
                // GPIO low (0) = LED on, high (1) = LED off
                if (matrix[row][col].current == 1) {
                    matrix[row][col].lys = false;
                }
                else{matrix[row][col].lys = true;}

                if (!matrix[row][col].lys) ++count;
            }
            // Deselect
            gpio_put(rowPins[row], 0);
            if (count >= numCols && setup > 17) done[row] = true;
            else {done[row] = false;}
        }
    }
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
    
    
    // Wait for magnets on rows 0 and 1
    // gripper();
    waitForInitialMagnets(matrix);
    gripper();
    moves = waitForMovesFromPC();
    waitForInitialMagnets(matrix);
    clearLights(matrix);

    while (true) {
        
        // Scan the matrix: set one row HIGH at a time.
        for (int row = 0; row < numRows; row++) {
            // Ensure all light pins are HIGH (LEDs off) before activating a row
            for (int j = 0; j < numCols; j++) {
                gpio_put(colPins_light[j], 1);
            }

            // Set all rows LOW first.
            for (int r = 0; r < numRows; r++) {
                gpio_put(rowPins[r], 0);
            }
            
            // Set the active row HIGH.
            gpio_put(rowPins[row], 1);
            
            sleep_us(100);
            // Read column inputs for the active row and update each point.
            for (int col = 0; col < numCols; col++) {
                runs++;
                // Control LED based on the matrix point's light status.
                if (matrix[row][col].lys == true) {
                    gpio_put(colPins_light[col], 0);
                } else {
                    gpio_put(colPins_light[col], 1);
                }

                sleep_us(50);  // Short delay for signal stabilization.

                // Save the previous value.
                matrix[row][col].last = matrix[row][col].current;
                // Read the new value.
                matrix[row][col].current = gpio_get(colPins[col]);

                // On input change after initial cycles
                if (runs >= 65 && matrix[row][col].current != matrix[row][col].last) {
                    sleep_us(1000);  // Debounce delay.
                    if (state == 1) {
                        first_col = matrix[row][col].col;
                        first_row = matrix[row][col].row;
                        std::vector<Move> legalMoves = getLegalMovesForSquare(moves, first_col, first_row);
                        for (const auto &move : legalMoves) {
                            matrix[move.destination.second][move.destination.first].lys = true;
                        }
                        state = 2;
                    }
                    else if (state == 2) {
                        second_col = matrix[row][col].col;
                        second_row = matrix[row][col].row;
                        
                        if (!(first_col == second_col && first_row == second_row) && matrix[second_row][second_col].lys == true) {
                            clearLights(matrix);
                            Move movemade = {{first_col, first_row}, {second_col, second_row}};
                            std::string movetosend = moveToString(movemade);
                            printf("%s", movetosend.c_str());
                            gripper();
                            moves = waitForMovesFromPC();
                            state = 1;
                            runs = 0;
                        }
                        else if ((first_col == second_col && first_row == second_row))
                        {
                            clearLights(matrix);
                            state = 1;
                            runs = 0;

                        }
                        else{
                            matrix[row][col].current = 0;
                            state = 2;
                        }
                       
                        
                    }
                }
                sleep_us(100);
            }
        }
    }
    
    return 0;
}
