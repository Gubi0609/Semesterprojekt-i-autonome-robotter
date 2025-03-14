#include <string>
#include <vector>

using namespace std;


class StockfishLinux {

    public:
        StockfishLinux();
        StockfishLinux(const string&);
        void appendMovesMade(const string&);
        string getBestMove(const string&);
        vector<string> getLegalMoves(const string&);
        ~StockfishLinux();

    private:
        int stockfishIn[2]; // Pipe for sending commands to Stockfish. Array of 2 integers.
        int stockfishOut[2]; // Pipe for reading output from Stockfish. Array of 2 integers.
        pid_t pid; // Process ID of the Stockfish engine.
        string movesMade;

        bool writeToStockfish(const string& command);
        void waitForReady();
        string readFromStockfish();

};