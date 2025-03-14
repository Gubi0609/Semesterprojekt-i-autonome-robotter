#include <iostream>
#include <vector>
#include <array>
#include <string>

// Function to convert a vector of chess move strings (like "a2a4") to a vector of numeric moves.
std::vector<std::array<int, 4>> convertMoves(const std::vector<std::string>& moves) {
    std::vector<std::array<int, 4>> converted;
    for (const auto& move : moves) {
        int startFile = move[0] - 'a';  // Convert 'a'->0, 'b'->1, etc.
        int startRank = move[1] - '1';  // Convert '1'->0, '2'->1, etc.
        int endFile   = move[2] - 'a';
        int endRank   = move[3] - '1';
        converted.push_back({startFile, startRank, endFile, endRank});
    }
    return converted;
}

// Function to find moves starting from a given file and rank.
std::vector<std::array<int, 4>> findMovesStartingAt(const std::vector<std::array<int, 4>>& numericMoves,
                                                     int file, int rank) {
    std::vector<std::array<int, 4>> result;
    for (const auto& move : numericMoves) {
        if (move[0] == file && move[1] == rank) {
            result.push_back(move);
        }
    }
    return result;
}

int main() {
    // Example vector of chess move strings.
    std::vector<std::string> moves = {"a2a4", "b2b4", "c1c5", "a2a3", "a2b3"};
    
    // Convert moves to numeric representation.
    std::vector<std::array<int, 4>> numericMoves = convertMoves(moves);
    
    // Input starting coordinates from the user.
    int file = 0, rank = 1;
    
    // Find all moves starting at the given coordinate.
    std::vector<std::array<int, 4>> movesFound = findMovesStartingAt(numericMoves, file, rank);
    
    // Output the result.
    if (movesFound.empty()) {
        std::cout << "No moves found starting from (" << file << ", " << rank << ").\n";
    } else {
        std::cout << "Moves starting from (" << file << ", " << rank << "):\n";
        for (const auto& move : movesFound) {
            std::cout << "(" << move[0] << ", " << move[1] << ") -> ("
                      << move[2] << ", " << move[3] << ")\n";
        }
    }
    
    return 0;
}
