#include "stockfishLinux.h"
#include <iostream>
#include <chrono>
#include <thread>


int main() {
#ifndef _WIN32
    std::string stockfishPath = "./stockfish-ubuntu-x86-64-sse41-popcnt";
#endif

    std::cout << "Stockfish path: " << stockfishPath << std::endl;

    StockfishLinux engine(stockfishPath, 5);
    std::cout << "Stockfish engine created" << std::endl;

    engine.openSerialPort("/dev/ttyACM0");
    

    while (true) {
        std::vector<std::string> legalMoves = engine.getLegalMoves();
        std::cout << "Legal moves: ";
        for (const std::string& move : legalMoves) {
            std::cout << move << " ";
        }
        std::cout << std::endl;
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        engine.sendlegelmoves(legalMoves);

        std::string player_move = engine.movefrompico();
        std::cout << "Player move: " << player_move << std::endl;
        engine.appendMovesMade(player_move);

        std::string bestMove = engine.getBestMove();
        std::cout << "Best move: " << bestMove << std::endl;
        engine.appendMovesMade(bestMove);
    }

    engine.closeSerialPort();

    return 0;
}

// g++ -o test10 main.cpp stockfishLinux.cpp -std=c++11 -pthread