#include <string>
#include <vector>

using namespace std;


class StockfishLinux {

    public:
        StockfishLinux();
        StockfishLinux(const string&, int);
        void appendMovesMade(const string&);
        string getBestMove();
        vector<string> getLegalMoves();
        ~StockfishLinux();

        // Metoder, der benytter serieporten (som er gemt i serial_fd)
        int sendlegelmoves(const std::vector<std::string>& legalMoves);
        std::string movefrompico();

        // Metoder til håndtering af serieporten
        int openSerialPort(const char* port);
        ssize_t writeSerialPort(const std::string& data);
        std::string readSerialPort();
        void closeSerialPort();

    private:
        int stockfishIn[2]; // Pipe for sending commands to Stockfish. Array of 2 integers.
        int stockfishOut[2]; // Pipe for reading output from Stockfish. Array of 2 integers.
        pid_t pid; // Process ID of the Stockfish engine.
        string movesMade;
        int difficulty;
        int serial_fd; // File descriptor for the serial port.

        bool writeToStockfish(const string& command);
        void waitForReady();
        string readFromStockfish();


};