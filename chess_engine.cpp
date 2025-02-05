// Prototype til skakbræt. Det skrives IKKE i OOP til at starte med, da jeg bare skal teste hvordan det virker.
// Jeg vil gerne have en funktion, der kan printe brættet ud, og en funktion, der kan flytte brikkerne rundt.
// Jeg vil også gerne have en funktion, der kan tjekke om et træk er lovligt.
// Jeg repræsenterer brikkerne gennem BitBoards.


#include <iostream>
#include <cstdint>
#include <string>

using namespace std;

bool whiteToMove = true;

// Bitboards for brikkerne
typedef uint64_t Bitboard;

Bitboard whitePawns = 0x000000000000FF00; // Rank 2
Bitboard whiteKnights = 0x00000000000042; // b1 og g1
Bitboard whiteBishops = 0x0000000000000024; // c1 og f1
Bitboard whiteRooks = 0x0000000000000081; // a1 og h1
Bitboard whiteQueens = 0x0000000000000008; // d1
Bitboard whiteKing = 0x0000000000000010; // e1

Bitboard blackPawns = 0x00FF000000000000; // Rank 7
Bitboard blackKnights = 0x4200000000000000; // b8 og g8
Bitboard blackBishops = 0x2400000000000000; // c8 og f8
Bitboard blackRooks = 0x8100000000000000; // a8 og h8
Bitboard blackQueens = 0x0800000000000000; // d8
Bitboard blackKing = 0x1000000000000000; // e8

// How to read bitboards:
// A bitboard is a 64-bit integer (uint64_t) where each bit represents a square on the chessboard.

// The least significant bit (LSB) (bit 0) represents a1 (bottom-left corner).
// The most significant bit (MSB) (bit 63) represents h8 (top-right corner).

// 8  56 57 58 59 60 61 62 63
// 7  48 49 50 51 52 53 54 55
// 6  40 41 42 43 44 45 46 47
// 5  32 33 34 35 36 37 38 39
// 4  24 25 26 27 28 29 30 31
// 3  16 17 18 19 20 21 22 23
// 2   8  9 10 11 12 13 14 15
// 1   0  1  2  3  4  5  6  7
//     a  b  c  d  e  f  g  h

// If a bit is 1, a piece is on that square.
// If a bit is 0, the square is empty.

// For example, the bitboard for white pawns is 0x00FF000000000000. 0x00FF000000000000 is a hexadecimal representation of the bitboard.
// In binary, 0x00FF000000000000 is 00000000 11111111 00000000 00000000 00000000 00000000 00000000 00000000:

// 8  0 0 0 0 0 0 0 0
// 7  0 0 0 0 0 0 0 0
// 6  0 0 0 0 0 0 0 0
// 5  0 0 0 0 0 0 0 0
// 4  0 0 0 0 0 0 0 0
// 3  0 0 0 0 0 0 0 0
// 2  1 1 1 1 1 1 1 1
// 1  0 0 0 0 0 0 0 0
//    a b c d e f g h

// 0x or 0X: The prefix 0x indicates that the number is hexadecimal. Both 0x and 0X are valid.
// 0xA3 represents the hexadecimal value A3.
// 0x7F represents the hexadecimal value 7F.
// 0 = 0000, 1 = 0001, 2 = 0010, 3 = 0011, 4 = 0100, 5 = 0101, 6 = 0110, 7 = 0111, 8 = 1000, 9 = 1001, A = 1010, B = 1011, C = 1100, D = 1101, E = 1110, F = 1111

int squareToBitIndex(const string square){
    // Function that converts a square to a bit index.

    char file = square[0];
    int row = square[1] - '0';

    int bitIndex = 8 *(row - 1) + (file - 'a');

    return bitIndex;
}

char getPieceAtSquare(const int bitSquare){

    Bitboard mask = 1ULL << bitSquare;

    if (whitePawns & mask) return 'P';
    if (whiteKnights & mask) return 'N';
    if (whiteBishops & mask) return 'B';
    if (whiteRooks & mask) return 'R';
    if (whiteQueens & mask) return 'Q';
    if (whiteKing & mask) return 'K';
    
    if (blackPawns & mask) return 'p';
    if (blackKnights & mask) return 'n';
    if (blackBishops & mask) return 'b';
    if (blackRooks & mask) return 'r';
    if (blackQueens & mask) return 'q';
    if (blackKing & mask) return 'k';

    return '.'; // Empty square
}

string checkOccupancy(const string square){
    // Function that checks if a square is occupied and returns the bitBoard of the piece on the square.

    int bitIndex = squareToBitIndex(square);
    Bitboard mask = 1ULL << bitIndex;

    if (whitePawns & mask) return "wP";
    if (whiteKnights & mask) return "wN";
    if (whiteBishops & mask) return "wB";
    if (whiteRooks & mask) return "wR";
    if (whiteQueens & mask) return "wQ";
    if (whiteKing & mask) return "wK";
    
    if (blackPawns & mask) return "bP";
    if (blackKnights & mask) return "bN";
    if (blackBishops & mask) return "bB";
    if (blackRooks & mask) return "bR";
    if (blackQueens & mask) return "bQ";
    if (blackKing & mask) return "bK";

    return ""; // Empty square

}

void printSingleBitboard(uint64_t &board) {
    for (int rank = 7; rank >= 0; rank--) {
        for (int file = 0; file < 8; file++) {
            int square = rank * 8 + file;
            cout << ((board >> square) & 1) << " ";
        }
        cout << "\n";
    }
    cout << "\n";
}

void printBoard(){

    for(int row = 7; row >= 0; row--){
        cout << (row + 1) << "| ";
        for(int file = 0; file < 8; file++){
            int bitSquare = row * 8 + file;
            cout << getPieceAtSquare(bitSquare) << " ";
        }
        cout << "|" << "\n";
    }

    cout << "  -----------------\n";
    cout << "   a b c d e f g h\n";

}


void movePiece(uint64_t &bitboard, string from, string to){
    // Function that moves a piece from one square to another.
    
    int fromIndex = squareToBitIndex(from); // Find the bitindex of the square the piece is moving from.
    int toIndex = squareToBitIndex(to); // Find the bitindex of the square the piece is moving to.

    bitboard &= ~(1ULL << fromIndex); // Remove piece from "from" square
    bitboard |= (1ULL << toIndex);    // Place piece on "to" square
}

int main() {

    string input;

    printBoard();
    movePiece(whiteKnights, "b1", "c3");
    printBoard();

    // while(true){
    // printBoard();
    // cout << (whiteToMove ? "White" : "Black") << " to move: ";
    // cin >> input;

    // string fromSquare = input.substr(0, 2);
    // string toSquare = input.substr(2, 2);

    // string pieceTypeToMove = checkOccupancy(fromSquare);

    // if(pieceTypeToMove != ""){
    //    if (pieceTypeToMove == "wP"){
    //     movePiece(whitePawns, fromSquare, toSquare);
    // }
    // else if (pieceTypeToMove == "wN"){
    //     movePiece(whiteKnights, fromSquare, toSquare);
    // }
    // else if (pieceTypeToMove == "wB"){
    //     movePiece(whiteBishops, fromSquare, toSquare);
    // }
    // else if (pieceTypeToMove == "wR"){
    //     movePiece(whiteRooks, fromSquare, toSquare);
    // }
    // else if (pieceTypeToMove == "wQ"){
    //     movePiece(whiteQueens, fromSquare, toSquare);
    // }
    // else if (pieceTypeToMove == "wK"){
    //     movePiece(whiteKing, fromSquare, toSquare);
    // }
    // else if (pieceTypeToMove == "bP"){
    //     movePiece(blackPawns, fromSquare, toSquare);
    // }
    // else if (pieceTypeToMove == "bN"){
    //     movePiece(blackKnights, fromSquare, toSquare);
    // }
    // else if (pieceTypeToMove == "bB"){
    //     movePiece(blackBishops, fromSquare, toSquare);
    // }
    // else if (pieceTypeToMove == "bR"){
    //     movePiece(blackRooks, fromSquare, toSquare);
    // }
    // else if (pieceTypeToMove == "bQ"){
    //     movePiece(blackQueens, fromSquare, toSquare);
    // }
    // else if (pieceTypeToMove == "bK"){
    //     movePiece(blackKing, fromSquare, toSquare);
    // }

    // whiteToMove = !whiteToMove;

    // }
    // else{
    //     cout << "Invalid move!" << "\n";
    // }
    // }

    return 0;
}