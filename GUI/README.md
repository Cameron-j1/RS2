Make sure you have installed the required packages:

Stockfish chess engine: sudo apt install stockfish

SFML graphic library: sudo apt install libsfml-dev

If you havent got boost, install by sudo apt install libboost-all-dev

To build, run: g++ chessAPI.cpp -o chess -lsfml-graphics -lsfml-window -lsfml-system -lboost_system -lboost_program_options -lpthread

To run: ./chess

![Screenshot from 2025-02-21 15-59-39](https://github.com/user-attachments/assets/c686e9f6-5685-4107-b0f0-89371f49dd3a)
