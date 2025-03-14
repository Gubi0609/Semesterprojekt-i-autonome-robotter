#include "stockfishLinux.h"
#include <iostream>

int main() {
#ifndef _WIN32
    std::string stockfishPath = "./stockfish-ubuntu-x86-64-sse41-popcnt";
#endif

    std::cout << "Stockfish path: " << stockfishPath << std::endl;

    StockfishLinux engine = StockfishLinux(stockfishPath);
    std::cout << "Stockfish engine created" << std::endl;

    engine.appendMovesMade("e2e4 e7e5");

    std::vector<std::string> legalMoves = engine.getLegalMoves();
    std::cout << "Legal moves: ";
    for (const std::string& move : legalMoves) {
        std::cout << move << " ";
    }
    std::cout << std::endl;

    std::string bestMove = engine.getBestMove();
    std::cout << "Best move: " << bestMove << std::endl;

    engine.~StockfishLinux();
    return 0;
}