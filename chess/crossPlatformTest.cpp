#include "stockfish.h"
#include <iostream>

int main() {
#ifdef _WIN32
    std::string stockfishPath = "stockfish-windows-x86-64-avx2.exe";
#else
    std::string stockfishPath = "./stockfish-ubuntu-x86-64-sse41-popcnt";
#endif

    std::cout << "Stockfish path: " << stockfishPath << std::endl;

    Stockfish* engine = createStockfishInstance(stockfishPath);

    std::string bestMove = engine->getBestMove("e2e4 e7e5");
    std::cout << "Best move: " << bestMove << std::endl;

    delete engine;
    return 0;
}