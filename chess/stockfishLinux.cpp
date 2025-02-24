#ifndef _WIN32

#include "stockfishLinux.h"
#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <cstring>
#include <sstream>

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
}

bool StockfishLinux::writeToStockfish(const string& command) {
    std::string cmd = command + "\n";
    return write(stockfishIn[1], cmd.c_str(), cmd.size()) != -1;
}

string StockfishLinux::readFromStockfish(){
    char buffer[256];
        std::string output;
        ssize_t bytesRead;

        while ((bytesRead = read(stockfishOut[0], buffer, sizeof(buffer) - 1)) > 0) {
            buffer[bytesRead] = '\0';
            output += buffer;
            if (output.find("bestmove") != std::string::npos) {
                break;
            }
        }

        // Extract the "bestmove" line
        std::string bestMove;
        std::istringstream stream(output);
        std::string line;
        while (std::getline(stream, line)) {
            if (line.find("bestmove") != std::string::npos) {
                std::istringstream bestMoveStream(line);
                std::string token;
                bestMoveStream >> token; // Skip "bestmove"
                bestMoveStream >> bestMove; // Get the actual move
                break;
            }
        }

        return bestMove;
}

string StockfishLinux::getBestMove(const string& position){
    writeToStockfish("position startpos moves " + position);
    writeToStockfish("go depth 20");
    return readFromStockfish();
}

StockfishLinux::~StockfishLinux() {
    writeToStockfish("quit");
    close(stockfishIn[1]);
    close(stockfishOut[0]);
    waitpid(pid, NULL, 0);
}

#endif