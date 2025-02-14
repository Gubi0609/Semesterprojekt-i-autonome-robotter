#ifndef STOCKFISH_H
#define STOCKFISH_H

#include <string>

class Stockfish {
public:
    virtual ~Stockfish() {}
    virtual std::string getBestMove(const std::string& position) = 0;
};

Stockfish* createStockfishInstance(const std::string& stockfishPath); // Factory function

#endif