#include <string>
#include <vector>

using namespace std;


class StockfishLinux {

    public:
        StockfishLinux();
        StockfishLinux(const string&);
        string getBestMove(const string&);
        vector<string> getLegalMoves(const string&);
        ~StockfishLinux();

    private:
        int stockfishIn[2]; 
        int stockfishOut[2];
        pid_t pid;

        bool writeToStockfish(const string& command);
        void waitForReady();
        string readFromStockfish();

};