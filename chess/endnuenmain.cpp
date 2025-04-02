#include "stockfishLinux.h"
#include <iostream>



int main() {
#ifndef _WIN32
    std::string stockfishPath = "./stockfish-ubuntu-x86-64-sse41-popcnt";
#endif

    std::cout << "Stockfish path: " << stockfishPath << std::endl;

    StockfishLinux engine(stockfishPath, 5);
    std::cout << "Stockfish engine created" << std::endl;

    engine.appendMovesMade(" ");

    while (true) {
        std::vector<std::string> legalMoves = engine.getLegalMoves();
        std::cout << "Legal moves: ";
        for (const std::string& move : legalMoves) {
            std::cout << move << " ";
        }
        std::cout << std::endl;

        engine.sendlegelmoves(legalMoves);

        std::string player_move = engine.movefrompico();
        engine.appendMovesMade(player_move);

        std::string bestMove = engine.getBestMove();
        std::cout << "Best move: " << bestMove << std::endl;
        engine.appendMovesMade(bestMove);
        
        sleep_us(1000)
    }

    return 0;
}