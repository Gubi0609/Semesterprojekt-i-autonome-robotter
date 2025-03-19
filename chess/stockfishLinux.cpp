#ifndef _WIN32 // Exclude the entire file from Windows builds

#include "stockfishLinux.h"
#include <iostream>
#include <unistd.h>     // For fork, pipe, read, write, close, dup2, STDIN_FILENO, STDOUT_FILENO
#include <sys/types.h>  // Defines types used in system calls (pid_t, etc.).
#include <sys/wait.h>   // For waitpid
#include <cstring>      // Provides functions like memcpy() and strcmp().
#include <sstream>      // For istringstream
#include <vector>
#include <limits>       // For numeric_limits

using namespace std;

StockfishLinux::StockfishLinux(const string& stockfishPath, int diff) {
    /* 
    Creates an instance of the Stockfish engine by forking a child process and
    establishing communication channels with the child process.

    :param stockfishPath: The path to the Stockfish executable.
    :param diff: The difficulty of the stockfish engine. 1 - 5.
    */

    movesMade = "";
    
    if (diff > 5) {
        difficulty = 5;
    }
    else if (diff < 1) {
        difficulty = 1;
    }
    else {
        difficulty = diff;
    }

    if (pipe(stockfishIn) == -1 || pipe(stockfishOut) == -1) { // Creates two pipes (stockfishIn and stockfishOut). Exits if pipes fail.
        perror("Pipe failed");
        exit(1);
    }

    pid = fork(); // Forks the process. Returns 0 for the child process, the child's PID for the parent, and -1 for failure.
    if (pid == 0) { // Child process
        dup2(stockfishIn[0], STDIN_FILENO); // Replaces the child's standard input (STDIN_FILENO) with stockfishIn[0].
        dup2(stockfishOut[1], STDOUT_FILENO); // Replaces the child's standard output (STDOUT_FILENO) with stockfishOut[1].
        close(stockfishIn[1]); // Closes the write end of stockfishIn for child.
        close(stockfishOut[0]); // Closes the read end of stockfishOut for child.
        execlp(stockfishPath.c_str(), stockfishPath.c_str(), NULL); // The child process replaces itself with Stockfish.
        perror("execlp failed"); // Prints an error message if execlp fails.
        exit(1);
    } else if (pid < 0) { // Fork failed
        perror("Fork failed");
        exit(1);
    }

    close(stockfishIn[0]); // Close the read end of stockfishIn for the parent.
    close(stockfishOut[1]); // Close the write end of stockfishOut for the parent.

    // Write "uci" and "isready" commands to Stockfish
    writeToStockfish("uci");
    writeToStockfish("isready");

    waitForReady(); // Wait for readyok response from Stockfish
}

void StockfishLinux::appendMovesMade(const string& moveMade) {
    /*
    Appends a move to the string of moves made. Handles pawn promotion if applicable.

    :param moveMade: The move made to append to movesMade. Only one move can be appended at a time.
    */

    if (moveMade.size() == 4) {
    string processedMove = handlePromotion(moveMade); // Check for pawn promotion
        if (!processedMove.empty()) {
            movesMade += " " + processedMove;
        }
    } else if (moveMade.size() == 5) {
        movesMade += " " + moveMade;
    }

}

bool StockfishLinux::writeToStockfish(const string& command) {
    /*
    writes a command to Stockfish.cmd
    
    :param command: The command to write to Stockfish.
    */

    string cmd = command + "\n";
    return write(stockfishIn[1], cmd.c_str(), cmd.size()) != -1; // write(fd, buffer, size) → Writes size bytes from buffer to file descriptor fd. != -1 → Checks for errors
}

void StockfishLinux::waitForReady() {
    /*
    Waits for stockfish to send readyok response. Is only to be used after sending isready command.
    */

    char buffer[4096];
    string output;
    ssize_t bytesRead; // (signed size type) stores the number of bytes read.

    // Read from the pipe until "readyok" is found
    while (true) {
        bytesRead = read(stockfishOut[0], buffer, sizeof(buffer) - 1); // Returns number of bytes read. Returns -1 on error. Returns 0 if end of file (EOF) is reached.
        if (bytesRead <= 0) {
            break;  // Exit if no data is read
        }

        buffer[bytesRead] = '\0'; // Null-terminate the buffer
        output += buffer; // Append the buffer to the output

        // Check for the "readyok" response to ensure Stockfish is ready
        if (output.find("readyok") != string::npos) {
            break;
        }
    }
}

string StockfishLinux::readFromStockfish() {
    char buffer[4096];
    string output;
    ssize_t bytesRead;
    string legalMoves;
    string bestMove;

    while ((bytesRead = read(stockfishOut[0], buffer, sizeof(buffer) - 1)) > 0) {
        buffer[bytesRead] = '\0';
        output += buffer;

        // Check for the presence of bestmove
        if (output.find("bestmove") != string::npos) {
            istringstream stream(output); // Treats a string as a stream, allowing easy extraction of words.
            string line;
            while (getline(stream, line)) { // Extracts lines from the stream
                if (line.find("bestmove") == 0) {
                    istringstream bestMoveStream(line);
                    string keyword;
                    bestMoveStream >> keyword >> bestMove; // Extracts the best move by reading the second word.
                    return bestMove;
                }
            }
        }

        // Parse the legal moves
        if (output.find(": 1") != string::npos) {
            istringstream stream(output);
            string line;
            while (getline(stream, line)) {
                if (line.find(":") != string::npos) {  // Look for lines that include ":"
                    string move = line.substr(0, line.find(':'));  // Extract the move before ":"
                    
                    // Check if the move is a valid chess move (like a2a3, b2b3, etc.)
                    if (move.size() == 4 && isalpha(move[0]) && isdigit(move[1]) &&
                        isalpha(move[2]) && isdigit(move[3])) {
                        
                        legalMoves += move + " "; // Append the move to the legalMoves string
                    } else if (move.size() == 5 && isalpha(move[0]) && isdigit(move[1]) &&
                        isalpha(move[2]) && isdigit(move[3]) && (move[4] == 'q' || move[4] == 'r' ||
                        move[4] == 'b' || move[4] == 'n')) {
                        
                        legalMoves += move + " "; // Append the move to the legalMoves string
                    }
                }
            }
        }

        if(output.find("Nodes searched") != string::npos){ // If the output contains "Nodes searched" end of legal moves is reached.
            return legalMoves;
        }
    }

    return ""; // Return empty string if no best move or legal move is found
}

string StockfishLinux::getFEN() {
    /*
    Gets the current FEN representation of the board from Stockfish.

    :return: FEN string of the current board position.
    */

    writeToStockfish("position startpos moves " + movesMade); // Set up position
    writeToStockfish("d"); // Ask for board state

    char buffer[4096];
    string output;
    ssize_t bytesRead;

    while ((bytesRead = read(stockfishOut[0], buffer, sizeof(buffer) - 1)) > 0) {
        buffer[bytesRead] = '\0';
        output += buffer;

        // Look for the line containing "Fen:"
        size_t fenPos = output.find("Fen: ");
        if (fenPos != string::npos) {
            size_t endLine = output.find("\n", fenPos);
            return output.substr(fenPos + 5, endLine - fenPos - 5); // Extract FEN part
        }
    }

    return ""; // Return empty if no FEN found
}

string StockfishLinux::handlePromotion(const string& move) {
    /*
    Ensures that only pawn moves trigger promotion handling by checking the board state.

    :param move: The move in UCI format (e.g., "e7e8").
    :return: The move with the promotion piece appended if applicable.
    */

    if (move.size() != 4) return move; // Ensure it's a standard move

    char startFile = move[0];  // First letter (file)
    char startRank = move[1];  // Second digit (rank)
    char endRank = move[3];    // Fourth digit (rank)

    // Get the FEN string from Stockfish
    string fen = getFEN();
    if (fen.empty()) return move; // FEN retrieval failed

    // Extract board position (first part of FEN before the first space)
    string boardState = fen.substr(0, fen.find(' '));

    // Convert rank and file to FEN index
    int fileIndex = startFile - 'a';   // 'a' = 0, 'b' = 1, ..., 'h' = 7
    int rankIndex = '8' - startRank;   // Rank 8 = index 0, Rank 7 = index 1, ..., Rank 1 = index 7

    // Find piece at (fileIndex, rankIndex) in FEN
    int boardPos = 0;
    char piece = ' ';

    for (char c : boardState) {
        if (isdigit(c)) {
            boardPos += (c - '0'); // Skip empty squares
        } else if (c == '/') {
            continue; // Skip row separators
        } else {
            if (boardPos == rankIndex * 8 + fileIndex) {
                piece = c; // Found the piece at the given position
                break;
            }
            boardPos++;
        }
    }

    // Only apply promotion if the piece is a pawn ('P' for white, 'p' for black)
    if (piece != 'P' && piece != 'p') {
        return move; // Not a pawn, return move unchanged
    }

    // Ensure move is from rank 7 -> 8 (white) or rank 2 -> 1 (black)
    if ((startRank == '7' && endRank == '8') || (startRank == '2' && endRank == '1')) {
        char promotionPiece = 'q'; // Default to queen

        // Ask user for promotion choice
        cout << "Pawn promotion detected! Choose piece (q, r, b, n): ";
        cin >> promotionPiece;

        // Ensure valid input
        if (promotionPiece != 'q' && promotionPiece != 'r' && 
            promotionPiece != 'b' && promotionPiece != 'n') {
            promotionPiece = 'q'; // Default to queen if invalid input
        }

        return move + promotionPiece; // Append promotion piece
    }

    return move; // Return unchanged if not a promotion move
}

bool StockfishLinux::isOccupied(const string& move) {
    /*
    Checks if the to square of the move is occupied by a piece.

    :param move: The move in UCI format (e.g., "e7e8").
    */

    char endFile = move[2]; // Get the file of the end square
    char endRank = move[3]; // Get the rank of the end square

    appendMovesMade(move); // Append the move to the moves made
    string fen = getFEN(); // Get the FEN representation of the board
    if (fen.empty()) {
        cerr << "Failed to retrieve FEN from Stockfish." << endl;
        return false; // Assume unoccupied if FEN retrieval fails
    }
    movesMade = movesMade.substr(0, movesMade.size() - move.size() - 1); // Remove the move from moves made again

    // Extract board position (first part of FEN before the first space)
    string boardState = fen.substr(0, fen.find(' '));

    // Convert rank and file to FEN index
    int fileIndex = endFile - 'a';   // 'a' = 0, 'b' = 1, ..., 'h' = 7
    int rankIndex = '8' - endRank;   // Rank 8 = index 0, Rank 7 = index 1, ..., Rank 1 = index 7

    int boardPos = 0;
    for (char c : boardState) {
        if (isdigit(c)) {
            boardPos += (c - '0'); // Skip empty squares
        } else if (c == '/') {
            continue; // Skip row separators
        } else {
            if (boardPos == rankIndex * 8 + fileIndex) {
                return true; // Found a piece at the target square
            }
            boardPos++;
        }
    }

    return false; // No piece found at the target square
}

string StockfishLinux::getBestMove(){
    /*
    Gets the best move from Stockfish for a given position.

    :param position: The position in moves made standard ("e2e4 a8a6 ...").
    */

    string depth = to_string(difficulty * 4);

    writeToStockfish("position startpos moves " + movesMade); // Set the position in Stockfish
    writeToStockfish("go depth " + depth); // Search for the best move with a depth of 20
    
    return readFromStockfish(); // Return the best move
}

vector<string> StockfishLinux::getLegalMoves(){
    /*
    Gets all the legal moves from Stockfish for a given position. Returns as a list of moves.getBestMove

    :param position: The position in moves made standard ("e2e4 a8a6 ...").
    */
    
    writeToStockfish("position startpos moves " + movesMade); // Set the position in Stockfish
    writeToStockfish("go perft 1"); // Get the legal moves using the perft command

    string legalMovesStr = readFromStockfish();  // Get space-separated moves
    vector<string> legalMoves;
    istringstream stream(legalMovesStr);
    string move;

    while (stream >> move) {  // Split string into moves
        legalMoves.push_back(move); // Add move to vector
    }

    return legalMoves;  // Return vector of moves
}

StockfishLinux::~StockfishLinux() {
    /*
    Closes the Stockfish pipes after end of use.
    */
    
    writeToStockfish("quit");
    close(stockfishIn[1]);
    close(stockfishOut[0]);
    waitpid(pid, NULL, 0); // Wait for the child process to terminate
}

#endif // end of _WIN32