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
#include "std_srvs/srv/trigger.hpp"

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

// Difficulty settings for Stockfish
enum class Difficulty {
    EASY,
    MEDIUM,
    HARD
};

// Default difficulty
Difficulty currentDifficulty = Difficulty::MEDIUM;

// Stockfish parameters for each difficulty level
struct StockfishParams {
    int searchDepth;
    int skill;  // Stockfish skill level (0-20)
    int moveTime; // Max time in milliseconds
};

std::unordered_map<Difficulty, StockfishParams> difficultyParams = {
    {Difficulty::EASY, {5, 5, 500}},      // Shallow depth, low skill, quick moves
    {Difficulty::MEDIUM, {10, 10, 1000}}, // Medium depth and skill
    {Difficulty::HARD, {15, 20, 2000}}    // Deep search, max skill, longer think time
};

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

// Dropdown UI for difficulty selection
class DropdownMenu {
public:
    sf::RectangleShape mainButton;
    sf::Text mainLabel;
    std::vector<sf::RectangleShape> optionButtons;
    std::vector<sf::Text> optionLabels;
    std::vector<std::string> optionNames;
    bool isOpen;
    int selectedIndex;
    sf::Font font;

    DropdownMenu(float x, float y, float width, float height, 
                 const sf::Font& font, const std::vector<std::string>& options) {
        this->font = font;
        this->optionNames = options;
        this->isOpen = false;
        this->selectedIndex = 1; // Default to MEDIUM

        // Setup main button
        mainButton.setSize(sf::Vector2f(width, height));
        mainButton.setPosition(x, y);
        mainButton.setFillColor(sf::Color(100, 100, 100));
        mainButton.setOutlineThickness(2);
        mainButton.setOutlineColor(sf::Color::Black);

        // Setup main label
        mainLabel.setFont(font);
        mainLabel.setString("Difficulty: " + options[selectedIndex]);
        mainLabel.setCharacterSize(18);
        mainLabel.setFillColor(sf::Color::White);
        sf::FloatRect textBounds = mainLabel.getLocalBounds();
        float labelX = x + 10;
        float labelY = y + (height - textBounds.height) / 2.0f - textBounds.top;
        mainLabel.setPosition(labelX, labelY);

        // Setup option buttons and labels
        for (int i = 0; i < options.size(); i++) {
            sf::RectangleShape optionButton;
            optionButton.setSize(sf::Vector2f(width, height));
            optionButton.setPosition(x, y + (i + 1) * height);
            optionButton.setFillColor(sf::Color(80, 80, 80));
            optionButton.setOutlineThickness(1);
            optionButton.setOutlineColor(sf::Color::Black);
            optionButtons.push_back(optionButton);

            sf::Text optionLabel;
            optionLabel.setFont(font);
            optionLabel.setString(options[i]);
            optionLabel.setCharacterSize(18);
            optionLabel.setFillColor(sf::Color::White);
            textBounds = optionLabel.getLocalBounds();
            labelX = x + 10;
            labelY = y + (i + 1) * height + (height - textBounds.height) / 2.0f - textBounds.top;
            optionLabel.setPosition(labelX, labelY);
            optionLabels.push_back(optionLabel);
        }
    }

    void draw(sf::RenderWindow& window) {
        window.draw(mainButton);
        window.draw(mainLabel);

        if (isOpen) {
            for (int i = 0; i < optionButtons.size(); i++) {
                window.draw(optionButtons[i]);
                window.draw(optionLabels[i]);
            }
        }
    }

    bool handleClick(sf::Vector2f mousePos) {
        if (mainButton.getGlobalBounds().contains(mousePos)) {
            isOpen = !isOpen;
            return true;
        }

        if (isOpen) {
            for (int i = 0; i < optionButtons.size(); i++) {
                if (optionButtons[i].getGlobalBounds().contains(mousePos)) {
                    selectedIndex = i;
                    mainLabel.setString("Difficulty: " + optionNames[i]);
                    isOpen = false;
                    sf::FloatRect textBounds = mainLabel.getLocalBounds();
                    float x = mainButton.getPosition().x + 10;
                    float y = mainButton.getPosition().y +
                              (mainButton.getSize().y - textBounds.height) / 2.0f - textBounds.top;
                    mainLabel.setPosition(x, y);
                    
                    // Update current difficulty
                    if (optionNames[i] == "EASY") currentDifficulty = Difficulty::EASY;
                    else if (optionNames[i] == "MEDIUM") currentDifficulty = Difficulty::MEDIUM;
                    else if (optionNames[i] == "HARD") currentDifficulty = Difficulty::HARD;
                    
                    return true;
                }
            }
        }
        return false;
    }
};

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

void publishFEN(
    std::shared_ptr<rclcpp::Node> node,
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> publisher, 
    bool blackTurn, 
    std::string castlingAvail, 
    int halfmoveClock, 
    int fullmoveNumer) {
    
    std_msgs::msg::String msg;
    msg.data = generateFullFEN(blackTurn ? 'b' : 'w', castlingAvail, halfmoveClock, fullmoveNumer);
    publisher->publish(msg);
    RCLCPP_INFO(node->get_logger(), "Published FEN: %s", msg.data.c_str());
}

void configureStockfish() {
    // Get parameters for current difficulty level
    StockfishParams params = difficultyParams[currentDifficulty];
    
    // Configure Stockfish with the appropriate parameters
    stockfish_in << "setoption name Skill Level value " << params.skill << "\n" << std::flush;
    
    // Additional options can be set here based on difficulty
    if (currentDifficulty == Difficulty::EASY) {
        // For easy mode, we can limit Stockfish's abilities
        stockfish_in << "setoption name Contempt value -50\n" << std::flush; // Makes Stockfish play more conservatively
    } else if (currentDifficulty == Difficulty::MEDIUM) {
        stockfish_in << "setoption name Contempt value 0\n" << std::flush;   // Neutral play
    } else if (currentDifficulty == Difficulty::HARD) {
        stockfish_in << "setoption name Contempt value 15\n" << std::flush;  // More aggressive play
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    std::string msgFromCamera;
    bool newMove = false;
    auto node = rclcpp::Node::make_shared("chessGUI",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    RCLCPP_INFO(node->get_logger(), "Starting chess GUI node...");
    auto publisher = node->create_publisher<std_msgs::msg::String>("/chess_moves", 10);

    auto subscriber = node->create_subscription<std_msgs::msg::String>(
        "/player_move", 10,
        [node, &msgFromCamera, &newMove](const std_msgs::msg::String::SharedPtr msg) {
            if (!newMove) {
                msgFromCamera = msg->data;
                newMove = true;
                RCLCPP_INFO(node->get_logger(), "Received from Camera: %s", msgFromCamera.c_str());
            }
        });

    // Subscribe to difficulty change requests
    auto difficultySubscriber = node->create_subscription<std_msgs::msg::String>(
        "/chess_difficulty", 10,
        [node](const std_msgs::msg::String::SharedPtr msg) {
            std::string difficultyStr = msg->data;
            if (difficultyStr == "DIFFICULTY_EASY") {
                currentDifficulty = Difficulty::EASY;
                RCLCPP_INFO(node->get_logger(), "Difficulty set to EASY");
            } else if (difficultyStr == "DIFFICULTY_MEDIUM") {
                currentDifficulty = Difficulty::MEDIUM;
                RCLCPP_INFO(node->get_logger(), "Difficulty set to MEDIUM");
            } else if (difficultyStr == "DIFFICULTY_HARD") {
                currentDifficulty = Difficulty::HARD;
                RCLCPP_INFO(node->get_logger(), "Difficulty set to HARD");
            }
            configureStockfish();
        });

    auto msg = std_msgs::msg::String();

    // Initialising stockfish communication via boost
    stockfish_in << "uci\n" << std::flush;
    std::string line;
    while (std::getline(stockfish_out, line)) {
        if (line == "uciok") {
            RCLCPP_INFO(node->get_logger(), "Stockfish initialized successfully!");
            break; // UCI initialization completed
        }
    }
    
    // Initial configuration of Stockfish based on default difficulty
    configureStockfish();

    sf::RenderWindow window(sf::VideoMode(BOARD_SIZE * SQUARE_SIZE, BOARD_SIZE * SQUARE_SIZE + 50), "Chess Game", sf::Style::Titlebar | sf::Style::Close);
    sf::Texture texture;
    texture.loadFromFile(chessPiecesPath);

    // Load font for the dropdown and status text
    sf::Font font;
    // Load your preferred font - replace with an appropriate path
    if (!font.loadFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load font!");
    }

    // Create difficulty dropdown menu
    DropdownMenu difficultyDropdown(10, BOARD_SIZE * SQUARE_SIZE + 10, 150, 30, font, {"EASY", "MEDIUM", "HARD"});
    
    // Status text for game information
    sf::Text statusText;
    statusText.setFont(font);
    statusText.setCharacterSize(16);
    statusText.setFillColor(sf::Color::White);
    statusText.setPosition(180, BOARD_SIZE * SQUARE_SIZE + 15);
    std::string status = "Game started. White to move.";

    // Change the fen string to set a different piece placement
    std::string fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR";
    window.setVerticalSyncEnabled(false);
    std::vector<std::string> pieceName;
    std::vector<sf::Sprite> pieces = makePieces(texture, fen);
    
    // publish the initial state of the chess board to the camera node for initialisation
    for (int i = 0; i < 8; i++) {
        for (int y = 0; y < 8; y++) {
            msg.data += board[i][y];
        }
    }

    publisher->publish(msg);

    auto fen_pub = node->create_publisher<std_msgs::msg::String>("/fen_string", 10);

    int lastClickedPiece = -1;
    int curRow = 0, curCol = 0;
    bool whiteCaptured = false, blackTurn = false;
    std::vector<std::vector<int>> moves;

    // Service: Reset Chessboard
    auto reset_service = node->create_service<std_srvs::srv::Trigger>(
        "/reset_chessboard",
        [&pieces, &texture, &fen, &publisher, &msgFromCamera, &newMove, &blackTurn, &lastClickedPiece, &fen_pub, &status, node]
        (const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            
            for (int r = 0; r < 8; ++r)
                for (int c = 0; c < 8; ++c)
                    board[r][c] = '-';

            pieces.clear(); // Clear old sprite objects
            fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR";
            pieces = makePieces(texture, fen);
    
            halfmoveClock = 0;
            fullmoveNumer = 1;
            castlingAvail = "KQkq";
            blackTurn = false;
            lastClickedPiece = -1;
            msgFromCamera.clear();
            newMove = false;
            isCastling.clear();
            potentialMove.clear();
            isPotentialMove.clear();
            status = "Game reset. White to move.";
    
            // Publish new board state
            std_msgs::msg::String msg;
            for (int i = 0; i < 8; i++) {
                for (int y = 0; y < 8; y++) {
                    msg.data += board[i][y];
                }
            }
            publisher->publish(msg);
            publishFEN(node, fen_pub, blackTurn, castlingAvail, halfmoveClock, fullmoveNumer);
            std::cout << "publish FEN" << std::endl;
    
            response->success = true;
            response->message = "Board reset to initial position.";
        }
    );

    // Game loop until window is closed
    while (window.isOpen() && rclcpp::ok()) {
        rclcpp::spin_some(node);
        sf::Event event;
        bool boardStateChanged = false;
        
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

                    // First click logic - selecting a piece
                    for (int i = 0; i < pieces.size(); i++) {
                        if (pieces[i].getGlobalBounds().contains(mousePosF)) {
                            // Find the clicked piece
                            if (lastClickedPiece == -1) {
                                // Selection logic - no board state change
                                curRow = (int(pieces[i].getPosition().y)-8)/SQUARE_SIZE;
                                curCol = (int(pieces[i].getPosition().x)-8)/SQUARE_SIZE;
                                foundPiece = true;
                                lastClickedPiece = i;
                                // Find possible moves
                                moves = getPossibleMoves(curRow, curCol);
                                isPotentialMove.assign(isPotentialMove.size() - 1, false);

                                for (const auto& move : moves) {
                                    isPotentialMove[move[0]*8 + move[1]] = true;
                                }
                            }
                            // Capture move logic
                            else if (lastClickedPiece != -1) {
                                int row = (int(pieces[i].getPosition().y)-8)/SQUARE_SIZE;
                                int col = (int(pieces[i].getPosition().x)-8)/SQUARE_SIZE;
                                
                                if (isPotentialMove[row*8+col] == true) {
                                    pieces[i].setPosition(5000.0f, 5000.0f);
                                    whiteCaptured = true;
                                    // Don't set boardStateChanged yet as the full move isn't complete
                                }
                                else {
                                    lastClickedPiece = -1;
                                }
                            }
                            break;
                        }
                    }
                    
                    // Second click to confirm the move
                    if (!foundPiece && lastClickedPiece != -1) {
                        for (int i = 0; i < potentialMove.size(); i++) {
                            // Move piece to selected square if it's a valid move
                            if (potentialMove[i].getGlobalBounds().contains(mousePosF) && isPotentialMove[i] == true) {
                                int nextRow = (i/8) * SQUARE_SIZE + 8, nextCol = (i%8) * SQUARE_SIZE + 8;
                                pieces[lastClickedPiece].setPosition(nextCol, nextRow);

                                if (board[curRow][curCol] == 'P' || whiteCaptured) {
                                    halfmoveClock = 0; whiteCaptured = false;
                                }
                                else halfmoveClock++;
                                
                                // Update the virtual chess board
                                board[i/8][i%8] = board[curRow][curCol];
                                board[curRow][curCol] = '-';
                                
                                // Handle castling
                                if (isCastling[i] == true) {
                                    int rookNextCol = i % 8 == 2 ? 3 : 5, rookCurCol = i % 8 == 2 ? 0 : 7;
                                    for (int y = 0; y < pieces.size(); y++) {
                                        if ((int(pieces[y].getPosition().y)-8)/SQUARE_SIZE == curRow && 
                                            (int(pieces[y].getPosition().x)-8)/SQUARE_SIZE == rookCurCol) {
                                            
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
                                boardStateChanged = true; // Board state has changed, publish FEN later
                                lastClickedPiece = -1;
                                break;
                            }
                            // Reset if clicked outside valid moves
                            else if (potentialMove[i].getGlobalBounds().contains(mousePosF) && isPotentialMove[i] == false) {
                                lastClickedPiece = -1;
                            }
                        }
                        // Reset castling check and potential moves display
                        for (auto& pair : isCastling) {
                            pair.second = false;
                        }
                        isPotentialMove.assign(isPotentialMove.size() - 1, false);
                    }
                }
            }
        }

        // Update the chess board with the real move from player (sent from camera node)
        if (newMove && !blackTurn) {
            newMove = false;
            // Process all pending moves
            while (msgFromCamera.length() > 0) {
                int physicalMove[4];
                for (int i = 0; i < 4; i++) {
                    physicalMove[i] = msgFromCamera[i] - '0';
                } 
                for (int i = 0; i < pieces.size(); i++) {
                    // Find the piece played
                    if ((int(pieces[i].getPosition().y)-8)/SQUARE_SIZE == physicalMove[0] &&
                        (int(pieces[i].getPosition().x)-8)/SQUARE_SIZE == physicalMove[1]) {
                        pieces[i].setPosition(physicalMove[3] * SQUARE_SIZE + 8, physicalMove[2] * SQUARE_SIZE + 8);
                        board[physicalMove[2]][physicalMove[3]] = board[physicalMove[0]][physicalMove[1]];
                        board[physicalMove[0]][physicalMove[1]] = '-';
                    }
                }
                msgFromCamera.erase(0, 4);
            }
            blackTurn = true;
            boardStateChanged = true; // Board state has changed, publish FEN
        }

        // Publish FEN only when board state has changed
        if (boardStateChanged) {
            publishFEN(node, fen_pub, blackTurn, castlingAvail, halfmoveClock, fullmoveNumer);
        }

        // Rendering logic
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
        
        // Stockfish's turn
        if (blackTurn) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            fen = "position fen " + generateFullFEN('b', castlingAvail, halfmoveClock, fullmoveNumer) + '\n'; 
            std::cout << fen;
            blackTurn = false;
            
            // Stockfish move logic...
            stockfish_in << fen << std::flush;
            stockfish_in << "go depth 10\n" << std::flush;
            char piecePlayed = ' ', pieceCaptured = ' ';
            char moveType = 'n';
            std::string stockfishMove; // Declare stockfishMove variable
            
            while (std::getline(stockfish_out, line)) {
                if (line.rfind("bestmove", 0) == 0) {
                    bool capture = false, pawn = false;
                    // Extract only the move part
                    std::istringstream iss(line);
                    std::string bestmove;
                    iss >> bestmove >> stockfishMove; // Skip "bestmove", get the move
                    
                    // Process Stockfish move...
                    std::vector<std::vector<int>> fishMoves = chessMoveToCoordinates(stockfishMove);
                    // Update the graphic to show stockfish's latest move
                    for (int i = 0; i < pieces.size(); i++) {
                        int row = (int(pieces[i].getPosition().y)-8)/SQUARE_SIZE;
                        int col = (int(pieces[i].getPosition().x)-8)/SQUARE_SIZE;
                        // Find the piece that stockfish want to play
                        if (row == fishMoves[0][0] && col == fishMoves[0][1]) {
                            for (int y = 0; y < pieces.size(); y++) {
                                int rowy = (int(pieces[y].getPosition().y)-8)/SQUARE_SIZE;
                                int coly = (int(pieces[y].getPosition().x)-8)/SQUARE_SIZE; 
                                // Remove the captured piece from the chessboard by setting its position somewhere far far away
                                if (rowy == fishMoves[1][0] && coly == fishMoves[1][1]) {
                                    pieces[y].setPosition(5000.0f, 5000.0f);
                                    pieceCaptured = board[fishMoves[1][0]][fishMoves[1][1]];
                                    moveType = 'x';
                                    capture = true;
                                    break;
                                }
                            }
                            pieces[i].setPosition(fishMoves[1][1] * SQUARE_SIZE + 8, fishMoves[1][0] * SQUARE_SIZE + 8);
                            board[fishMoves[1][0]][fishMoves[1][1]] = board[row][col];
                            piecePlayed = board[row][col];

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
                    msg.data = stockfishMove + moveType + piecePlayed; // Using stockfishMove instead of undefined move
                    if (capture) msg.data += pieceCaptured;
                    break;
                }
            }
            
            // Publish the move to control node
            publisher->publish(msg);
            
            // Publish FEN after Stockfish move
            publishFEN(node, fen_pub, blackTurn, castlingAvail, halfmoveClock, fullmoveNumer);
            
            // Publish board state to camera node
            std::string boardStr;
            for (int i = 0; i < 8; i++) {
                for (int y = 0; y < 8; y++) {
                    boardStr += board[i][y];
                }
            }
            std_msgs::msg::String boardMsg;
            boardMsg.data = boardStr;
            publisher->publish(boardMsg);
        }
    }
    
    stockfish_in << "quit\n" << std::flush;
    stockfish.wait(); // Wait for the process to finish
    rclcpp::shutdown();
    return 0;
}