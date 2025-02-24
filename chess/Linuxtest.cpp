#include "stockfishLinux.h"
#include <iostream>

int main() {
#ifndef _WIN32
    std::string stockfishPath = "./stockfish-ubuntu-x86-64-sse41-popcnt";
#endif

    std::cout << "Stockfish path: " << stockfishPath << std::endl;

    StockfishLinux engine = StockfishLinux(stockfishPath);

    std::string bestMove = engine.getBestMove("e2e4 e7e5");
    std::cout << "Best move: " << bestMove << std::endl;

    engine.~StockfishLinux();
    return 0;
}