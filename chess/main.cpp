#include "stockfishLinux.h"
#include "SerialPort.h"
#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>

int main() {
    const char* port = "/dev/ttyACM0"; 
    SerialPort serial;

    if (serial.openSerial(port) < 0) {
        return 1;
    }

    std::cout << "Serial port opened successfully." << std::endl;
    
#ifndef _WIN32
    std::string stockfishPath = "./stockfish-ubuntu-x86-64-sse41-popcnt";
    
#endif

    std::cout << "Stockfish path: " << stockfishPath << std::endl;

    StockfishLinux engine(stockfishPath, 5);
    std::cout << "Stockfish engine created" << std::endl;

    for (int i = 0; i < 10; i++) {
        std::vector<std::string> legalMoves = engine.getLegalMoves();
        std::cout << "Legal moves: ";
        for (const std::string& move : legalMoves) {
            std::cout << move << " ";
        }
        std::cout << std::endl;

        std::this_thread::sleep_for(std::chrono::microseconds(100));
        serial.sendlegalmoves(legalMoves);

        std::string player_move = serial.movefrompico();
        std::cout << "Player move: " << player_move << std::endl;
        engine.appendMovesMade(player_move);

        std::string bestMove = engine.getBestMove();
        std::cout << "Best move: " << bestMove << std::endl;
        engine.appendMovesMade(bestMove);
    }

    serial.closeSerial();
    return 0;
}
// g++ -std=c++11 -o test5 main.cpp SerialPort.cpp stockfishLinux.cpp -pthread
