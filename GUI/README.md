Make sure you have installed the required packages:

Stockfish chess engine: sudo apt install stockfish

SFML graphic library: sudo apt install libsfml-dev

To build, run: g++ chessAPI.cpp -o chess -lsfml-graphics -lsfml-window -lsfml-system

To run: ./chess
