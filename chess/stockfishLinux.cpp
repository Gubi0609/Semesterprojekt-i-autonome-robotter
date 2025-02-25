#ifndef _WIN32

#include "stockfishLinux.h"
#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <cstring>
#include <sstream>
#include <vector>

using namespace std;

StockfishLinux::StockfishLinux(const string& stockfishPath) {
    if (pipe(stockfishIn) == -1 || pipe(stockfishOut) == -1) {
        perror("Pipe failed");
        exit(1);
    }

    pid = fork();
    if (pid == 0) { // Child process
        dup2(stockfishIn[0], STDIN_FILENO);
        dup2(stockfishOut[1], STDOUT_FILENO);
        close(stockfishIn[1]);
        close(stockfishOut[0]);
        execlp(stockfishPath.c_str(), stockfishPath.c_str(), NULL);
        perror("execlp failed");
        exit(1);
    } else if (pid < 0) {
        perror("Fork failed");
        exit(1);
    }

    close(stockfishIn[0]);
    close(stockfishOut[1]);

    // Write "uci" and "isready" commands to Stockfish
    writeToStockfish("uci");
    writeToStockfish("isready");

    waitForReady();
}

bool StockfishLinux::writeToStockfish(const string& command) {
    string cmd = command + "\n";
    return write(stockfishIn[1], cmd.c_str(), cmd.size()) != -1;
}

void StockfishLinux::waitForReady() {
    char buffer[4096];  // A larger buffer to avoid truncation
    string output;
    ssize_t bytesRead;

    // Collect output from Stockfish until it's ready
    while (true) {
        bytesRead = read(stockfishOut[0], buffer, sizeof(buffer) - 1);
        if (bytesRead <= 0) {
            break;  // Exit if no data is read
        }

        buffer[bytesRead] = '\0';
        output += buffer;

        // Check for the "readyok" response to ensure Stockfish is ready
        if (output.find("readyok") != string::npos) {
            break;  // Exit the loop when Stockfish signals readiness
        }
    }
}

string StockfishLinux::readFromStockfish() {
    char buffer[4096];  // A larger buffer to avoid truncation
    string output;
    ssize_t bytesRead;
    string legalMoves;
    string bestMove;

    // Collect full output first
    cout << "Reading from Stockfish ..." << endl;
    while ((bytesRead = read(stockfishOut[0], buffer, sizeof(buffer) - 1)) > 0) {
        buffer[bytesRead] = '\0';
        output += buffer;
        // Check for the presence of bestmove
        if (output.find("bestmove") != string::npos) {
            istringstream stream(output);
            string line;
            while (getline(stream, line)) {
                if (line.find("bestmove") == 0) {
                    istringstream bestMoveStream(line);
                    string keyword;
                    bestMoveStream >> keyword >> bestMove;
                    return bestMove;  // Return immediately once found
                }
            }
        }

        // Parse the legal moves
        if (output.find(": 1") != string::npos) {
            istringstream stream(output);
            string line;
            while (getline(stream, line)) {
                if (line.find(":") != string::npos) {  // Look for lines that look like "a2a3"
                    string move = line.substr(0, line.find(':'));  // Extract the move before ":"
                    
                    // Check if the move is a valid chess move (like a2a3, b2b3, etc.)
                    if (move.size() == 4 && isalpha(move[0]) && isdigit(move[1]) &&
                        isalpha(move[2]) && isdigit(move[3])) {
                        legalMoves += move + " ";
                    }
                }
            }
        }

        if(output.find("Nodes searched") != string::npos){
            return legalMoves;
        }
    }

    cout << "Finished reading from Stockfish" << endl;
    return "";
}

string StockfishLinux::getBestMove(const string& position){
    writeToStockfish("position startpos moves " + position);
    writeToStockfish("go depth 20");
    
    return readFromStockfish();
}

vector<string> StockfishLinux::getLegalMoves(const string& position){
    writeToStockfish("position startpos moves " + position);
    writeToStockfish("go perft 1");
    cout << "Searching for legal moves ..." << endl;

    string legalMovesStr = readFromStockfish();  // Get space-separated moves
    vector<string> legalMoves;
    istringstream stream(legalMovesStr);
    string move;

    while (stream >> move) {  // Split string into moves
        legalMoves.push_back(move);
    }

    return legalMoves;  // Return vector of moves
}

StockfishLinux::~StockfishLinux() {
    writeToStockfish("quit");
    close(stockfishIn[1]);
    close(stockfishOut[0]);
    waitpid(pid, NULL, 0);
}

#endif