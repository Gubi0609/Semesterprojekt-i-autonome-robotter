#ifndef _WIN32

#include "Stockfish.h"
#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <cstring>

class StockfishLinux : public Stockfish {
private:
    int stockfishIn[2], stockfishOut[2];
    pid_t pid;

    bool writeToStockfish(const std::string& command) {
        std::string cmd = command + "\n";
        return write(stockfishIn[1], cmd.c_str(), cmd.size()) != -1;
    }

    std::string readFromStockfish() {
        char buffer[256];
        std::string output;
        ssize_t bytesRead;
        
        while ((bytesRead = read(stockfishOut[0], buffer, sizeof(buffer) - 1)) > 0) {
            buffer[bytesRead] = '\0';
            output += buffer;
            if (output.find("bestmove") != std::string::npos) break;
        }
        return output;
    }

public:
    StockfishLinux(const std::string& stockfishPath) {
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

    std::string getBestMove(const std::string& position) override {
        writeToStockfish("position startpos moves " + position);
        writeToStockfish("go depth 20");
        return readFromStockfish();
    }

    ~StockfishLinux() {
        writeToStockfish("quit");
        close(stockfishIn[1]);
        close(stockfishOut[0]);
        waitpid(pid, NULL, 0);
    }
};

Stockfish* createStockfishInstance(const std::string& stockfishPath) {
    return new StockfishLinux(stockfishPath);
}

#endif