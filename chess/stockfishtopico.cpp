#include "stockfishLinux.h"
#include <iostream>



int main() {
#ifndef _WIN32
    std::string stockfishPath = "./stockfish-ubuntu-x86-64-sse41-popcnt";
    std::vector<std::string> all_moves = " ";
#endif

    std::cout << "Stockfish path: " << stockfishPath << std::endl;

    StockfishLinux engine = StockfishLinux(stockfishPath);
    std::cout << "Stockfish engine created" << std::endl;

    std::vector<std::string> legalMoves = engine.getLegalMoves(all_moves);
    std::cout << "Legal moves: ";

    while (true) {
        for (std::string move : legalMoves) {
            std::cout << move << " ";
        }
        std::cout << std::endl;

        std::string bestMove = engine.getBestMove(all_moves);
        std::cout << "Best move: " << bestMove << std::endl;
    }
}