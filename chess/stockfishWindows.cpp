#ifdef _WIN32

#include "stockfish.h"
#include <windows.h>
#include <iostream>


class StockfishWindows : public Stockfish {
    private:
        HANDLE hStdOutRead, hStdOutWrite, hStdInRead, hStdInWrite;
        PROCESS_INFORMATION pi;
    
        bool writeToStockfish(const std::string& command) {
            std::string cmd = command + "\n";
            DWORD bytesWritten;
            return WriteFile(hStdInWrite, cmd.c_str(), cmd.length(), &bytesWritten, NULL);
        }
    
        std::string readFromStockfish(bool waitForBestMove = false) {
            char buffer[256];
            DWORD bytesRead;
            std::string output;
            
            while (ReadFile(hStdOutRead, buffer, sizeof(buffer) - 1, &bytesRead, NULL) && bytesRead > 0) {
                buffer[bytesRead] = '\0';
                output += buffer;
                std::cout << buffer; // Debug output
    
                // If waiting for "bestmove", ensure we don't stop early
                if (!waitForBestMove && (output.find("uciok") != std::string::npos || output.find("readyok") != std::string::npos)) {
                    break;
                }
                if (waitForBestMove && output.find("bestmove") != std::string::npos) {
                    break;
                }
            }
            return output;
        }
    
    public:
        StockfishWindows(const std::string& stockfishPath) {
            SECURITY_ATTRIBUTES saAttr = {sizeof(SECURITY_ATTRIBUTES), NULL, TRUE};
    
            CreatePipe(&hStdOutRead, &hStdOutWrite, &saAttr, 0);
            SetHandleInformation(hStdOutRead, HANDLE_FLAG_INHERIT, 0);
            CreatePipe(&hStdInRead, &hStdInWrite, &saAttr, 0);
            SetHandleInformation(hStdInWrite, HANDLE_FLAG_INHERIT, 0);
    
            STARTUPINFOA si = {sizeof(STARTUPINFOA)};
            si.dwFlags = STARTF_USESTDHANDLES;
            si.hStdOutput = hStdOutWrite;
            si.hStdError = hStdOutWrite;
            si.hStdInput = hStdInRead;
    
            if (!CreateProcessA(stockfishPath.c_str(), NULL, NULL, NULL, TRUE, 0, NULL, NULL, &si, &pi)) {
                std::cerr << "Failed to start Stockfish. Error: " << GetLastError() << std::endl;
                exit(1);
            }
            CloseHandle(hStdOutWrite);
            CloseHandle(hStdInRead);
        }
    
        std::string getBestMove(const std::string& position) override {
            writeToStockfish("position startpos moves " + position);
            writeToStockfish("go depth 20");

            std::string response = readFromStockfish(true);
            size_t pos = response.find("bestmove");
            if (pos != std::string::npos) {
                std::string bestMove = response.substr(pos + 9, 5);
                return bestMove.substr(0, bestMove.find("\n"));
            }

            return "error";
        }
    
        ~StockfishWindows() {
            writeToStockfish("quit");
            CloseHandle(hStdOutRead);
            CloseHandle(hStdInWrite);
            WaitForSingleObject(pi.hProcess, INFINITE);
            CloseHandle(pi.hProcess);
            CloseHandle(pi.hThread);
        }
    };
    
    Stockfish* createStockfishInstance(const std::string& stockfishPath) {
        return new StockfishWindows(stockfishPath);
    }
    
    #endif