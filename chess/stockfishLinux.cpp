#ifndef _WIN32 // Exclude the entire file from Windows builds

#include "stockfishLinux.h"
#include <iostream>
#include <unistd.h>     // For fork, pipe, read, write, close, dup2, STDIN_FILENO, STDOUT_FILENO
#include <sys/types.h>  // Defines types used in system calls (pid_t, etc.).
#include <sys/wait.h>   // For waitpid
#include <cstring>      // Provides functions like memcpy() and strcmp().
#include <sstream>      // For istringstream
#include <vector>
#include <fcntl.h>
#include <termios.h>

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
    Appends a move to the string of moves made.

    :param moveMade: The move made to append to movesMade. Multiple moves can be appended at once by seperating with space.
    */

    movesMade += " " + moveMade;

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

int StockfishLinux::openSerialPort(const char* port) {
    serial_fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0) {
        perror("Fejl ved åbning af serialport");
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(serial_fd, &tty) != 0) {
        perror("Fejl ved tcgetattr");
        close(serial_fd);
        serial_fd = -1;
        return -1;
    }
    
    // Sæt baudrate til 115200
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);


    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;             
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 1;         // Bloker indtil mindst 1 byte modtages
    tty.c_cc[VTIME] = 0;         // Ingen timeout - vent uendeligt

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;     // Ingen hardware flow control

    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        perror("Fejl ved tcsetattr");
        close(serial_fd);
        serial_fd = -1;
        return -1;
    }
    return serial_fd;
}


ssize_t StockfishLinux::writeSerialPort(const std::string& data) {
    if (serial_fd < 0) {
        std::cerr << "Serieporten er ikke åben!" << std::endl;
        return -1;
    }
    ssize_t bytesWritten = write(serial_fd, data.c_str(), data.size());
    if (bytesWritten < 0) {
        perror("Fejl ved skrivning til serialport");
    }
    return bytesWritten;
}

std::string StockfishLinux::readSerialPort() {
    if (serial_fd < 0) {
        std::cerr << "Serieporten er ikke åben!" << std::endl;
        return "";
    }
    char buffer[256];
    ssize_t bytesRead = read(serial_fd, buffer, sizeof(buffer) - 1);
    if (bytesRead < 0) {
        perror("Fejl ved læsning fra serialport");
        return "";
    }
    buffer[bytesRead] = '\0';
    return std::string(buffer);
}

void StockfishLinux::closeSerialPort() {
    if (serial_fd >= 0) {
        close(serial_fd);
        serial_fd = -1;
    }
}


int StockfishLinux::sendlegelmoves(const std::vector<std::string>& legalMoves) {
    std::string legalMovesString;
    for (const auto& move : legalMoves) {
        legalMovesString += move + " ";
    }
    if (!legalMovesString.empty() && legalMovesString.back() == ' ') {
        legalMovesString.pop_back();
    }
    legalMovesString += "\n";

    ssize_t bytesWritten = writeSerialPort(legalMovesString);
    if (bytesWritten < 0) {
        return 1;
    } else {
        std::cout << "Sendte legal moves string: " << legalMovesString;
    }
    return 0;
}

std::string StockfishLinux::movefrompico() {
    while (true) {
        std::string move = readSerialPort();
        if (move.size() >= 4) {
            std::cout << "Data modtaget fra porten: " << move << std::endl;
            return move;
        }
        std::cout << "Ugyldig pakke registreret: " << move << std::endl;
    }
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