#include <iostream>
#include <cstdio>
#include <vector>
#include <cstring>
#include <algorithm>
#include <string>
#include <cctype>
#include <unordered_map>
#include <sstream>
#include <thread>
#include <chrono>
#include <boost/process.hpp>
#include <SFML/Graphics.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace bp = boost::process;

bp::ipstream stockfish_out; // Capture Stockfish output
bp::opstream stockfish_in;  // Send commands to Stockfish
bp::child stockfish("stockfish", bp::std_out > stockfish_out, bp::std_in < stockfish_in);

const int BOARD_SIZE = 8;
const int SQUARE_SIZE = 80;  // Adjust as needed
std::string chessPiecesPath = ament_index_cpp::get_package_share_directory("ur3_test_control") + "/images/pieces.png";

// Colors for the chessboard
sf::Color cream(240, 217, 181);
sf::Color brown(181, 136, 99);

unsigned char board[8][8] = {
    {'-', '-', '-', '-', '-', '-', '-', '-'},
    {'-', '-', '-', '-', '-', '-', '-', '-'},
    {'-', '-', '-', '-', '-', '-', '-', '-'},
    {'-', '-', '-', '-', '-', '-', '-', '-'},
    {'-', '-', '-', '-', '-', '-', '-', '-'},
    {'-', '-', '-', '-', '-', '-', '-', '-'},
    {'-', '-', '-', '-', '-', '-', '-', '-'},
    {'-', '-', '-', '-', '-', '-', '-', '-'}
};

std::vector<sf::RectangleShape> potentialMove;
std::vector<bool> isPotentialMove;
std::unordered_map<int, bool> isCastling;

int halfmoveClock = 0, fullmoveNumer = 1;

std::string castlingAvail = "KQkq";

bool isValid(int row, int col) {
    return row >= 0 && row < 8 && col >= 0 && col < 8;
}

// Helper function to check if two pieces are enemies (one uppercase, one lowercase)
bool isEnemy(char piece1, char piece2) {
    return (std::isupper(piece1) && std::islower(piece2)) || 
           (std::islower(piece1) && std::isupper(piece2));
}

std::vector<std::vector<int>> getPossibleMoves(int row, int col) {
    std::vector<std::vector<int>> moves;
    char currentPiece = board[row][col];
    bool isWhite = std::isupper(currentPiece);

    if (!isValid(row, col) || currentPiece == '-') {
        return moves; // Invalid position or empty cell
    }

    if (currentPiece == 'p' || currentPiece == 'P') {
        int direction = isWhite ? -1 : 1; // White moves up, Black moves down
        int startRow = isWhite ? 6 : 1;   // Starting row for two-step move

        // Forward move
        int newRow = row + direction;
        if (isValid(newRow, col) && board[newRow][col] == '-') {
            moves.push_back({newRow, col});
            // Two-step move from starting row
            if (row == startRow && board[newRow + direction][col] == '-') {
                moves.push_back({newRow + direction, col});
            }
        }

        // Capture moves (diagonal)
        for (int dc : {-1, 1}) {
            newRow = row + direction;
            int newCol = col + dc;
            if (isValid(newRow, newCol) && board[newRow][newCol] != '-' && 
                isEnemy(currentPiece, board[newRow][newCol])) {
                moves.push_back({newRow, newCol});
            }
        }
    }
    else if (currentPiece == 'r' || currentPiece == 'R') {
        // Directions: up, down, left, right
        std::vector<std::vector<int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
        for (const auto& dir : directions) {
            int newRow = row, newCol = col;
            while (true) {
                newRow += dir[0];
                newCol += dir[1];
                if (!isValid(newRow, newCol)) break;
                if (board[newRow][newCol] == '-') {
                    moves.push_back({newRow, newCol});
                }
                else if (isEnemy(currentPiece, board[newRow][newCol])) {
                    moves.push_back({newRow, newCol});
                    break; // Stop after capturing
                }
                else {
                    break; // Blocked by friendly piece
                }
            }
        }
    }
    else if (currentPiece == 'n' || currentPiece == 'N') {
        std::vector<std::vector<int>> offsets = {
            {-2, -1}, {-2, 1}, {-1, -2}, {-1, 2},
            {1, -2}, {1, 2}, {2, -1}, {2, 1}
        };
        for (const auto& offset : offsets) {
            int newRow = row + offset[0];
            int newCol = col + offset[1];
            if (isValid(newRow, newCol) && 
                (board[newRow][newCol] == '-' || isEnemy(currentPiece, board[newRow][newCol]))) {
                moves.push_back({newRow, newCol});
            }
        }
    }
    else if (currentPiece == 'b' || currentPiece == 'B') {
        std::vector<std::vector<int>> directions = {{-1, -1}, {-1, 1}, {1, -1}, {1, 1}};
        for (const auto& dir : directions) {
            int newRow = row, newCol = col;
            while (true) {
                newRow += dir[0];
                newCol += dir[1];
                if (!isValid(newRow, newCol)) break;
                if (board[newRow][newCol] == '-') {
                    moves.push_back({newRow, newCol});
                }
                else if (isEnemy(currentPiece, board[newRow][newCol])) {
                    moves.push_back({newRow, newCol});
                    break;
                }
                else {
                    break;
                }
            }
        }
    }
    else if (currentPiece == 'q' || currentPiece == 'Q') {
        std::vector<std::vector<int>> directions = {
            {-1, 0}, {1, 0}, {0, -1}, {0, 1}, // Rook-like
            {-1, -1}, {-1, 1}, {1, -1}, {1, 1} // Bishop-like
        };
        for (const auto& dir : directions) {
            int newRow = row, newCol = col;
            while (true) {
                newRow += dir[0];
                newCol += dir[1];
                if (!isValid(newRow, newCol)) break;
                if (board[newRow][newCol] == '-') {
                    moves.push_back({newRow, newCol});
                }
                else if (isEnemy(currentPiece, board[newRow][newCol])) {
                    moves.push_back({newRow, newCol});
                    break;
                }
                else {
                    break;
                }
            }
        }
    }
    else if (currentPiece == 'k' || currentPiece == 'K') {
        // Standard king moves
        std::vector<std::vector<int>> offsets = {
            {-1, -1}, {-1, 0}, {-1, 1},
            {0, -1},           {0, 1},
            {1, -1},  {1, 0},  {1, 1}
        };
        for (const auto& offset : offsets) {
            int newRow = row + offset[0];
            int newCol = col + offset[1];
            if (isValid(newRow, newCol) && 
                (board[newRow][newCol] == '-' || isEnemy(currentPiece, board[newRow][newCol]))) {
                moves.push_back({newRow, newCol});
            }
        }

        // Castling moves
        bool isWhite = (currentPiece == 'K');
        int startRow = isWhite ? 7 : 0; // White king at row 7, Black at row 0
        
        if (row == startRow && col == 4) { // King must be at e1 (7,4) or e8 (0,4)
            // Kingside castling (right, toward h-file)
            if (board[startRow][7] == (isWhite ? 'R' : 'r') && // Rook at h1/h8
                board[startRow][5] == '-' && board[startRow][6] == '-') { // f and g empty
                moves.push_back({startRow, 6}); // King moves to g1/g8
                isCastling[startRow * 8 + 6] = true;
            }
            // Queenside castling (left, toward a-file)
            if (board[startRow][0] == (isWhite ? 'R' : 'r') && // Rook at a1/a8
                board[startRow][1] == '-' && board[startRow][2] == '-' && board[startRow][3] == '-') { // b, c, d empty
                moves.push_back({startRow, 2}); // King moves to c1/c8
                isCastling[startRow * 8 + 2] = true;
            }
        }
    }

    return moves;
}

std::vector<std::vector<int>> chessMoveToCoordinates(const std::string& move) {

    // Chessboard columns are 'a' to 'h' -> 0 to 7
    // Chessboard rows are '1' to '8' -> 0 to 7 (8 is at the top of the board)

    // Extract the column and row from the move
    char start_col = move[0]; // 'e'
    char start_row = move[1]; // '2'
    char end_col = move[2];   // 'e'
    char end_row = move[3];   // '4'

    // Convert the columns ('a' to 'h') to index (0 to 7)
    int start_col_index = start_col - 'a'; // 'e' -> 4
    int end_col_index = end_col - 'a';     // 'e' -> 4

    // Convert rows ('1' to '8') to index (0 to 7)
    int start_row_index = 8 - (start_row - '0'); // '2' -> 6 (8 - 2)
    int end_row_index = 8 - (end_row - '0');     // '4' -> 4 (8 - 4)

    // Output the coordinates
    return {{start_row_index, start_col_index}, {end_row_index, end_col_index}};
}

void drawBoard(sf::RenderWindow &window) {
    for (int row = 0; row < BOARD_SIZE; row++) {
        for (int col = 0; col < BOARD_SIZE; col++) {
            sf::RectangleShape square(sf::Vector2f(SQUARE_SIZE, SQUARE_SIZE));
            sf::RectangleShape potentialMoveSquare(sf::Vector2f(SQUARE_SIZE, SQUARE_SIZE));

            square.setPosition(col * SQUARE_SIZE, row * SQUARE_SIZE);
            potentialMoveSquare.setPosition(col * SQUARE_SIZE, row * SQUARE_SIZE);
            potentialMoveSquare.setFillColor(sf::Color(255, 0, 0, 0));
            potentialMove.push_back(potentialMoveSquare);
            isPotentialMove.push_back(false);

            if ((row + col) % 2 == 0)
                square.setFillColor(cream);
            else
                square.setFillColor(brown);

            window.draw(square);
        }
    }
}

std::string generateFullFEN(char activeColor, 
                            std::string castling, int halfmoveClock, 
                            int fullmoveNumber) {
    std::stringstream fen;
    for (int row = 0; row < 8; ++row) {
        int emptyCount = 0;
        for (int col = 0; col < 8; ++col) {
            char square = board[row][col];
            if (square == '-') {
                ++emptyCount;
            } else {
                if (emptyCount > 0) {
                    fen << emptyCount;
                    emptyCount = 0;
                }
                fen << square;
            }
        }
        if (emptyCount > 0) {
            fen << emptyCount;
        }
        if (row < 7) {
            fen << '/';
        }
    }
    fen << " " << activeColor << " " << (castling.empty() ? "-" : castling) 
        << " - " << halfmoveClock << " " << fullmoveNumber; // En passant always "-"
    return fen.str();
}

std::vector<sf::Sprite> makePieces(const sf::Texture& texture, std::string fen) {
    std::vector<sf::Sprite> result;
    // Sprite size is 60 by 60
    int row = 0, col = 0;
    sf::Sprite piece(texture);
    for (int i = 0; i < fen.length(); i++) {
        switch (fen[i])
        {
        case 'r':
            piece.setTextureRect(sf::IntRect(128, 0, 64, 64));
            break;

        case 'n':
            piece.setTextureRect(sf::IntRect(192, 0, 64, 64));
            break;
        
        case 'b':
            piece.setTextureRect(sf::IntRect(256, 0, 64, 64));
            break;

        case 'q':
            piece.setTextureRect(sf::IntRect(64, 0, 64, 64));
            break;

        case 'k':
            piece.setTextureRect(sf::IntRect(0, 0, 64, 64));
            break;

        case 'p':
            piece.setTextureRect(sf::IntRect(320, 0, 64, 64));
            break;

        case 'R':
            // std::cout << "found RRR \n";
            piece.setTextureRect(sf::IntRect(128, 64, 64, 64));
            break;

        case 'N':
            piece.setTextureRect(sf::IntRect(192, 64, 64, 64));
            break;
        
        case 'B':
            piece.setTextureRect(sf::IntRect(256, 64, 64, 64));
            break;

        case 'Q':
            piece.setTextureRect(sf::IntRect(64, 64, 64, 64));
            break;

        case 'K':
            piece.setTextureRect(sf::IntRect(0, 64, 64, 64));
            break;

        case 'P':
            piece.setTextureRect(sf::IntRect(320, 64, 64, 64));
            break;
        
        case '1':
            col++; continue;
        
        case '2':
            col += 2; continue;

        case '3':
            col += 3; continue;

        case '4':
            col += 4; continue;

        case '5':
            col += 5; continue;

        case '6':
            col += 6; continue;

        case '7':
            col += 7; continue;
        default:
            break;
        }

        if (fen[i] != '/' && fen[i] != '8') {
            piece.setPosition(col * SQUARE_SIZE + 8, row * SQUARE_SIZE + 8);
            result.push_back(piece);
            board[row][col] = fen[i];
            col++;
        }
        
        if (fen[i] == '/') {
            row++;
            col = 0;
            continue;
        } 
    }
    return result;
}

void logBoard() {
    for (int i = 0; i < 8; i++) {
        for (int y = 0; y < 8; y++) {
            std::cout << board[i][y] << ' ';
        }
        std::cout << '\n';
    }
    std::cout << '\n';
}

int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("chessGUI",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    RCLCPP_INFO(node->get_logger(), "Starting chess GUI node...");
    auto publisher = node->create_publisher<std_msgs::msg::String>("/chess_moves", 10);
    auto msg = std_msgs::msg::String();

    // Initialising stockfish communication via boost
    stockfish_in << "uci\n" << std::flush;
    std::string line;
    while (std::getline(stockfish_out, line)) {
        if (line == "uciok") {
            RCLCPP_INFO(node->get_logger(), "Stockfish gud af!");
            break; // UCI initialization completed
        }
    }

    sf::RenderWindow window(sf::VideoMode(BOARD_SIZE * SQUARE_SIZE, BOARD_SIZE * SQUARE_SIZE), "Chessy", sf::Style::Titlebar | sf::Style::Close);
    sf::Texture texture;
    texture.loadFromFile(chessPiecesPath);

    // Change the fen string to set a different piece placement
    std::string fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR";

    window.setVerticalSyncEnabled(false);
    std::vector<std::string> pieceName;
    std::vector<sf::Sprite> pieces = makePieces(texture, fen);
    int lastClickedPiece = -1;
    int curRow = 0, curCol = 0;
    bool whiteCaptured = false, blackTurn = false;
    std::vector<std::vector<int>> moves;

    // Game loop until window is closed
    while (window.isOpen() && rclcpp::ok()) {
        sf::Event event;
        // Human to move (check if any mouse click was detected)
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();

            if (event.type == sf::Event::MouseButtonPressed) {
                if (event.mouseButton.button == sf::Mouse::Left && !blackTurn) {
                    sf::Vector2i mousePos = sf::Mouse::getPosition(window);
                    // Convert to float for bounds checking
                    sf::Vector2f mousePosF(static_cast<float>(mousePos.x), static_cast<float>(mousePos.y));
                    bool foundPiece = false;

                    // First mouse click, to select the pieces and display possible moves for that piece
                    for (int i = 0; i < pieces.size(); i++) {
                        if (pieces[i].getGlobalBounds().contains(mousePosF)) {
                            // Find the clicked piece
                            if (lastClickedPiece == -1) {
                                curRow = (int(pieces[i].getPosition().y)-8)/SQUARE_SIZE;
                                curCol = (int(pieces[i].getPosition().x)-8)/SQUARE_SIZE;
                                foundPiece = true;
                                lastClickedPiece = i;
                                // Find the possible moves for that piece and display them 
                                moves = getPossibleMoves(curRow, curCol);
                                isPotentialMove.assign(isPotentialMove.size() - 1, false);

                                for (const auto& move : moves) {
                                    isPotentialMove[move[0]*8 + move[1]] = true;
                                }
                            }
                            // This is the capture move, the piece will be remove if it's an enemy piece
                            else if (lastClickedPiece != -1) {
                                int row = (int(pieces[i].getPosition().y)-8)/SQUARE_SIZE;
                                int col = (int(pieces[i].getPosition().x)-8)/SQUARE_SIZE;
                                
                                if (isPotentialMove[row*8+col] == true) {
                                    pieces[i].setPosition(5000.0f, 5000.0f);
                                    whiteCaptured = true;
                                }
                                else {
                                    lastClickedPiece = -1;
                                }
                            }
                            break;
                        }
                    }
                    
                    // Second click to confirm next move
                    if (!foundPiece && lastClickedPiece != -1) {
                        for (int i = 0; i < potentialMove.size(); i++) {
                            // Find the square within possible move that was clicked on and move the piece to that square
                            if (potentialMove[i].getGlobalBounds().contains(mousePosF) && isPotentialMove[i] == true) {
                                int nextRow = (i/8) * SQUARE_SIZE + 8, nextCol = (i%8) * SQUARE_SIZE + 8;
                                pieces[lastClickedPiece].setPosition(nextCol, nextRow);

                                if (board[curRow][curCol] == 'P' || whiteCaptured) {
                                    halfmoveClock = 0; whiteCaptured = false;
                                }
                                else halfmoveClock++;
                                // Update the virtue chess board
                                board[i/8][i%8] = board[curRow][curCol];
                                board[curRow][curCol] = '-';
                                // Check if the detected move is the castling move
                                // If it is then move the right rook to the right position.
                                // That side will no longer be allow to castle (Maybe a boolean to check for this condition ?)
                                if (isCastling[i] == true) {
                                    int rookNextCol = i % 8 == 2 ? 3 : 5, rookCurCol = i % 8 == 2 ? 0 : 7;
                                    for (int y = 0; y < pieces.size(); y++) {
                                        if ((int(pieces[y].getPosition().y)-8)/SQUARE_SIZE == curRow && (int(pieces[y].getPosition().x)-8)/SQUARE_SIZE == rookCurCol) {
                                            pieces[y].setPosition(rookNextCol * SQUARE_SIZE + 8, curRow * SQUARE_SIZE + 8);
                                            castlingAvail.erase(std::remove(castlingAvail.begin(), castlingAvail.end(), 'K'), castlingAvail.end());
                                            castlingAvail.erase(std::remove(castlingAvail.begin(), castlingAvail.end(), 'Q'), castlingAvail.end());
                                            if (castlingAvail.length() == 0) castlingAvail = "-";
                                            board[curRow][rookNextCol] = board[curRow][rookCurCol];
                                            board[curRow][rookCurCol] = '-';
                                            break;
                                        }
                                    }
                                }
                                blackTurn = true;
                                lastClickedPiece = -1;
                                break;
                            }
                            // If the user clicks somewhere not within the possible moves, it will reset.
                            else if (potentialMove[i].getGlobalBounds().contains(mousePosF) && isPotentialMove[i] == false) {
                                lastClickedPiece = -1;
                            }
                        }
                        // Remove the temporary castling check
                        for (auto& pair : isCastling) {
                            pair.second = false;
                        }
                        // The red square used to display potential move will be removed
                        isPotentialMove.assign(isPotentialMove.size() - 1, false);
                    }

                }
            }
        }

        window.clear();
        drawBoard(window);
        for (int i = 0; i < pieces.size(); i++) {
            window.draw(pieces[i]);
        }
        for (int i = 0; i < isPotentialMove.size(); i++) {
            if (isPotentialMove[i] == true) {
                potentialMove[i].setFillColor(sf::Color(255, 0, 0, 100));
                window.draw(potentialMove[i]);
            }
        }
        window.display();
        char moveType = 'n';
        // // This is the Stockfish turn to play
        if (blackTurn) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Sleep for 0.5 seconds
            fen = "position fen " + generateFullFEN('b', castlingAvail, halfmoveClock, fullmoveNumer) + '\n'; 
            std::cout << fen;
            blackTurn = false;
            stockfish_in << fen << std::flush;
            stockfish_in << "go depth 10\n" << std::flush;

            while (std::getline(stockfish_out, line)) {
                if (line.rfind("bestmove", 0) == 0) {
                    bool capture = false, pawn = false;
                    // Extract only the move part
                    std::istringstream iss(line);
                    std::string bestmove;
                    std::string move;
                    iss >> bestmove >> move; // Skip "bestmove", get the move
                    // std::cout << move << '\n';
                    std::vector<std::vector<int>> fishMoves = chessMoveToCoordinates(move);
                    // Update the graphic to show stockfish's latest move
                    for (int i = 0; i < pieces.size(); i++) {
                        int row = (int(pieces[i].getPosition().y)-8)/SQUARE_SIZE;
                        int col = (int(pieces[i].getPosition().x)-8)/SQUARE_SIZE;
                        // Find the piece that stockfish want to play
                        if (row == fishMoves[0][0] && col == fishMoves[0][1]) {
                            for (int y = 0; y < pieces.size(); y++) {
                                int rowy = (int(pieces[y].getPosition().y)-8)/SQUARE_SIZE;
                                int coly = (int(pieces[y].getPosition().x)-8)/SQUARE_SIZE; 
                                if (rowy == fishMoves[1][0] && coly == fishMoves[1][1]) {
                                    pieces[y].setPosition(5000.0f, 5000.0f);
                                    capture = true;
                                    break;
                                }
                            }
                            pieces[i].setPosition(fishMoves[1][1] * SQUARE_SIZE + 8, fishMoves[1][0] * SQUARE_SIZE + 8);
                            board[fishMoves[1][0]][fishMoves[1][1]] = board[row][col];

                            if (board[row][col] == 'p') pawn = true;
                            // Castling check and move the correct rooks
                            if (board[row][col] == 'k' && abs(fishMoves[0][1] - fishMoves[1][1]) > 1) {
                                moveType = 'c';
                                int rookNextCol = fishMoves[1][1] == 2 ? 3 : 5, rookCurCol = fishMoves[1][1] == 2 ? 0 : 7;
                                // Move rooks and update the board
                                for (int y = 0; y < pieces.size(); y++) {
                                    if ((int(pieces[y].getPosition().y)-8)/SQUARE_SIZE == row && (int(pieces[y].getPosition().x)-8)/SQUARE_SIZE == rookCurCol) {
                                        pieces[y].setPosition(rookNextCol * SQUARE_SIZE + 8, row * SQUARE_SIZE + 8);
                                        castlingAvail.erase(std::remove(castlingAvail.begin(), castlingAvail.end(), 'k'), castlingAvail.end());
                                        castlingAvail.erase(std::remove(castlingAvail.begin(), castlingAvail.end(), 'q'), castlingAvail.end());
                                        if (castlingAvail.length() == 0) castlingAvail = "-";
                                        board[row][rookNextCol] = board[row][rookCurCol];
                                        board[row][rookCurCol] = '-';
                                        break;
                                    }
                                }
                            }

                            board[row][col] = '-';
                            break;
                        }
                    }
                    if (capture || pawn) halfmoveClock = 0; else halfmoveClock++; 
                    fullmoveNumer++;
                    msg.data = move + moveType;
                    break; // Stop after printing the move
                }
            }
            publisher->publish(msg);
            // logBoard();
            for (int i = 0; i < 8; i++) {
                std::string row;
                for (int y = 0; y < 8; y++) {
                    row += std::string(1, board[i][y]) + " "; // Convert char to string and add space
                }
                RCLCPP_INFO(node->get_logger(), "%s", row.c_str()); // Log each row
            }
            RCLCPP_INFO(node->get_logger(), ""); // Empty line for newline
        }
    }
    
    stockfish_in << "quit\n" << std::flush;
    stockfish.wait(); // Wait for the process to finish
    rclcpp::shutdown();
    return 0;
}
