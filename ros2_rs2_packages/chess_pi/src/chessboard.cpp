#pragma region Includes and Configuration

#include <iostream>
#include <SFML/Graphics.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <vector>
#include <string>
#include <sstream>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>

// Configs
const int BOARD_SIZE = 8;
const int SQUARE_SIZE = 45;
const int BOARD_OFFSET_X = 25;
const int BOARD_OFFSET_Y = 20;
const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 480;

// GUI State enum
enum GUIState {
    STARTUP_SCREEN,
    GAME_SCREEN
};

// Colors
sf::Color cream(211, 211, 211); // GREY
sf::Color brown(70, 130, 180);  // LIGHT BLUE
sf::Color buttonBlue(70, 130, 180);
sf::Color buttonHover(100, 160, 250);
sf::Color buttonText(255, 255, 255);
sf::Color dropdownBg(230, 230, 230);
sf::Color dropdownText(50, 50, 50);
sf::Color startButtonGreen(46, 139, 87); // Green color for start button
sf::Color startButtonHover(60, 179, 113); // Lighter green for hover
sf::Color warningRed(255, 50, 50); // Red color for warning flashing text

// Texture path
std::string chessPiecesPath = ament_index_cpp::get_package_share_directory("chess_pi") + "/images/pieces.png";

#pragma endregion Includes and Configuration

#pragma region Global Variables and Structs

// Flashing text struct
struct FlashingText {
   std::vector<sf::Text> textLines;
    bool isActive = false;
    bool isVisible = true;
    float frequency = 2.0f; // Flashes per second
    float timer = 0.0f;
};

// Global variables for move indication
sf::Vector2i selectedPiece(-1, -1);
std::vector<std::vector<int>> possibleMoves;
std::vector<sf::CircleShape> moveIndicators;

// Game state variables for complete FEN notation
bool whiteToMove = true;  // Track whose turn it is
bool whiteCanCastleKing = true;
bool whiteCanCastleQueen = true;
bool blackCanCastleKing = true;
bool blackCanCastleQueen = true;
std::string enPassantTarget = "-";  // En passant target square
int halfmoveClock = 0;  // Halfmove clock (moves since last pawn move or capture)
int fullmoveNumber = 1;  // Fullmove number

std::unordered_map<int, bool> isCastling;
unsigned char board[8][8] = {{'-'}};

#pragma endregion Global Variables and Structs

#pragma region Function Declarations

// Function declarations (add before any function definitions)
void updateMoveIndicators(int row, int col);
std::string coordsToChessNotation(int row, int col);

#pragma endregion Function Declarations

#pragma region Chess Logic Functions

// Function to generate FEN string from current board state
std::string boardToFen() {
    std::string fen = "";
    
    // 1. Board position
    for (int row = 0; row < 8; row++) {
        if (row > 0) fen += "/";
        
        int emptyCount = 0;
        for (int col = 0; col < 8; col++) {
            char piece = board[row][col];
            if (piece == '-') {
                emptyCount++;
            } else {
                if (emptyCount > 0) {
                    fen += std::to_string(emptyCount);
                    emptyCount = 0;
                }
                fen += piece;
            }
        }
        if (emptyCount > 0) {
            fen += std::to_string(emptyCount);
        }
    }
    
    // 2. Active color
    fen += " ";
    fen += whiteToMove ? "w" : "b";
    
    // 3. Castling availability
    fen += " ";
    std::string castling = "";
    if (whiteCanCastleKing) castling += "K";
    if (whiteCanCastleQueen) castling += "Q";
    if (blackCanCastleKing) castling += "k";
    if (blackCanCastleQueen) castling += "q";
    if (castling.empty()) castling = "-";
    fen += castling;
    
    // 4. En passant target square
    fen += " ";
    fen += enPassantTarget;
    
    // 5. Halfmove clock
    fen += " ";
    fen += std::to_string(halfmoveClock);
    
    // 6. Fullmove number
    fen += " ";
    fen += std::to_string(fullmoveNumber);
    
    return fen;
}

// Function to get just the board position part (for sending to other GUI)
std::string getBoardPosition() {
    std::string boardPos = "";
    
    for (int row = 0; row < 8; row++) {
        if (row > 0) boardPos += "/";
        
        int emptyCount = 0;
        for (int col = 0; col < 8; col++) {
            char piece = board[row][col];
            if (piece == '-') {
                emptyCount++;
            } else {
                if (emptyCount > 0) {
                    boardPos += std::to_string(emptyCount);
                    emptyCount = 0;
                }
                boardPos += piece;
            }
        }
        if (emptyCount > 0) {
            boardPos += std::to_string(emptyCount);
        }
    }
    
    return boardPos;
}

// Function to execute a move on the board and return move information
struct MoveInfo {
    bool isCapture;
    bool isCastling;
    bool isPromotion;
    char captured;
};

MoveInfo executeMove(int fromRow, int fromCol, int toRow, int toCol) {
    MoveInfo moveInfo = {false, false, false, '-'};
    
    char piece = board[fromRow][fromCol];
    char capturedPiece = board[toRow][toCol];
    
    // Check if it's a capture
    if (capturedPiece != '-') {
        moveInfo.isCapture = true;
        moveInfo.captured = capturedPiece;
    }
    
    // Check if it's a pawn promotion
    if ((piece == 'P' && toRow == 0) || (piece == 'p' && toRow == 7)) {
        moveInfo.isPromotion = true;
        std::cout << "Pawn promotion detected!" << std::endl;
    }
    
    board[fromRow][fromCol] = '-';
    
    // Handle promotion - convert pawn to queen
    if (moveInfo.isPromotion) {
        if (piece == 'P') {
            board[toRow][toCol] = 'Q';  // White pawn promotes to white queen
            std::cout << "White pawn promoted to Queen!" << std::endl;
        } else {
            board[toRow][toCol] = 'q';  // Black pawn promotes to black queen
            std::cout << "Black pawn promoted to Queen!" << std::endl;
        }
    } else {
        board[toRow][toCol] = piece;
    }
    
    // Handle castling
    if ((piece == 'K' || piece == 'k') && abs(toCol - fromCol) == 2) {
        // This is castling
        moveInfo.isCastling = true;
        if (toCol == 6) { // Kingside castling
            // Move rook from h-file to f-file
            char rook = board[fromRow][7];
            board[fromRow][7] = '-';
            board[fromRow][5] = rook;
            std::cout << "Kingside castling executed!" << std::endl;
        } else if (toCol == 2) { // Queenside castling
            // Move rook from a-file to d-file
            char rook = board[fromRow][0];
            board[fromRow][0] = '-';
            board[fromRow][3] = rook;
            std::cout << "Queenside castling executed!" << std::endl;
        }
    }
    
    // Update castling rights
    if (piece == 'K') {
        whiteCanCastleKing = false;
        whiteCanCastleQueen = false;
    } else if (piece == 'k') {
        blackCanCastleKing = false;
        blackCanCastleQueen = false;
    } else if (piece == 'R') {
        if (fromRow == 7 && fromCol == 7) whiteCanCastleKing = false;
        if (fromRow == 7 && fromCol == 0) whiteCanCastleQueen = false;
    } else if (piece == 'r') {
        if (fromRow == 0 && fromCol == 7) blackCanCastleKing = false;
        if (fromRow == 0 && fromCol == 0) blackCanCastleQueen = false;
    }
    
    // Update halfmove clock
    if (piece == 'P' || piece == 'p' || capturedPiece != '-') {
        halfmoveClock = 0;  // Reset on pawn move or capture
    } else {
        halfmoveClock++;
    }
    
    // Update fullmove number (increment after black's move)
    if (!whiteToMove) {
        fullmoveNumber++;
    }
    
    // Switch turns
    whiteToMove = !whiteToMove;
    
    // Reset en passant target (simplified - would need more logic for actual en passant)
    enPassantTarget = "-";
    
    return moveInfo;
}

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

#pragma endregion Chess Logic Functions

#pragma region GUI Structs and Classes

// Custom GUI Code for move validation

// Custom RoundedRectangleShape class for rounded buttons
class RoundedRectangleShape : public sf::Shape
{
public:
    RoundedRectangleShape(const sf::Vector2f& size = sf::Vector2f(0, 0), float radius = 0, unsigned int cornerPointCount = 8)
    : m_size(size), m_radius(radius), m_cornerPointCount(cornerPointCount)
    {
        update();
    }

    void setSize(const sf::Vector2f& size)
    {
        m_size = size;
        update();
    }

    const sf::Vector2f& getSize() const
    {
        return m_size;
    }

    void setRadius(float radius)
    {
        m_radius = radius;
        update();
    }

    float getRadius() const
    {
        return m_radius;
    }

    void setCornerPointCount(unsigned int count)
    {
        m_cornerPointCount = count;
        update();
    }

    virtual std::size_t getPointCount() const
    {
        return 4 * m_cornerPointCount;
    }

    virtual sf::Vector2f getPoint(std::size_t index) const
    {
        if (index >= getPointCount())
            return sf::Vector2f(0, 0);

        // Calculate which corner we're dealing with
        std::size_t cornerNumber = index / m_cornerPointCount;
        std::size_t cornerIndex = index % m_cornerPointCount;
        
        // Define corner centers
        sf::Vector2f centers[4] = {
            {m_radius, m_radius},                              // top-left
            {m_size.x - m_radius, m_radius},                   // top-right
            {m_size.x - m_radius, m_size.y - m_radius},        // bottom-right
            {m_radius, m_size.y - m_radius}                    // bottom-left
        };
        
        // Define start angles for each corner (in radians)
        float startAngles[4] = {
            180.0f * 3.14159f / 180.0f,  // top-left: start at 180 degrees
            270.0f * 3.14159f / 180.0f,  // top-right: start at 270 degrees
            0.0f * 3.14159f / 180.0f,    // bottom-right: start at 0 degrees
            90.0f * 3.14159f / 180.0f    // bottom-left: start at 90 degrees
        };
        
        // Calculate the angle for this specific point
        float angle = startAngles[cornerNumber] + cornerIndex * (90.0f * 3.14159f / 180.0f) / (m_cornerPointCount - 1);
        
        // Calculate position relative to corner center
        float x = centers[cornerNumber].x + std::cos(angle) * m_radius;
        float y = centers[cornerNumber].y + std::sin(angle) * m_radius;
        
        return sf::Vector2f(x, y);
    }

private:
    sf::Vector2f m_size;
    float m_radius;
    unsigned int m_cornerPointCount;
};

// Button and Dropdown structs
struct Button
{
    RoundedRectangleShape shape;
    sf::Text label;
    std::string name;
    bool isHovered = false;
};

struct Dropdown
{
    sf::RectangleShape mainButton;
    sf::Text mainLabel;
    std::vector<sf::RectangleShape> options;
    std::vector<sf::Text> optionLabels;
    std::vector<std::string> optionNames;
    bool isOpen = false;
    int selectedIndex = 0;
};

// Button creation function with color parameter
Button createButton(sf::Font &font, const std::string &label, float x, float y, float width, float height, sf::Color color, sf::Color hoverColor)
{
    Button button;
    button.shape.setSize(sf::Vector2f(width, height));
    button.shape.setFillColor(color);
    button.shape.setOutlineThickness(1);
    button.shape.setOutlineColor(sf::Color(40, 40, 40));
    button.shape.setPosition(x, y);
    
    // Set rounded corners radius (adjust as needed)
    button.shape.setRadius(8.0f);
    
    button.label.setFont(font);
    button.label.setString(label);
    button.label.setCharacterSize(16);
    button.label.setFillColor(buttonText);
    sf::FloatRect textBounds = button.label.getLocalBounds();
    button.label.setPosition(
        x + (width - textBounds.width) / 2.0f - textBounds.left,
        y + (height - textBounds.height) / 2.0f - textBounds.top);
    button.name = label;
    return button;
}

std::vector<Button> createGameButtons(sf::Font &font)
{
    std::vector<Button> buttons;
    std::vector<std::string> labels = {};

    int buttonWidth = 120, buttonHeight = 40;
    int rightPanelX = BOARD_OFFSET_X + BOARD_SIZE * SQUARE_SIZE + 280;
    
    std::cout << "Creating game buttons at X position: " << rightPanelX << std::endl;

    for (int i = 0; i < labels.size(); i++)
    {
        Button button = createButton(font, labels[i], rightPanelX, BOARD_OFFSET_Y + 40 + i * (buttonHeight + 15), 
                                     buttonWidth, buttonHeight, buttonBlue, buttonHover);
        buttons.push_back(button);
    }

    return buttons;
}

Dropdown createDifficultyDropdown(sf::Font &font)
{
    Dropdown dropdown;
    std::vector<std::string> options = {"Easy", "Medium", "Hard"};
    int dropdownWidth = 200, dropdownHeight = 40;
    // Center the dropdown in the window
    int centerX = (WINDOW_WIDTH - dropdownWidth) / 2 + 150;
    int startY = WINDOW_HEIGHT / 2;

    dropdown.mainButton.setSize(sf::Vector2f(dropdownWidth, dropdownHeight));
    dropdown.mainButton.setFillColor(dropdownBg);
    dropdown.mainButton.setOutlineThickness(1);
    dropdown.mainButton.setOutlineColor(sf::Color(180, 180, 180));
    dropdown.mainButton.setPosition(centerX, startY);
    dropdown.mainLabel.setFont(font);
    dropdown.mainLabel.setString("Difficulty: " + options[0]);
    dropdown.mainLabel.setCharacterSize(14);
    dropdown.mainLabel.setFillColor(dropdownText);
    sf::FloatRect textBounds = dropdown.mainLabel.getLocalBounds();
    dropdown.mainLabel.setPosition(centerX + 10, startY + (dropdownHeight - textBounds.height) / 2.0f - textBounds.top);

    for (int i = 0; i < options.size(); i++)
    {
        sf::RectangleShape option;
        option.setSize(sf::Vector2f(dropdownWidth, dropdownHeight));
        option.setFillColor(dropdownBg);
        option.setOutlineThickness(1);
        option.setOutlineColor(sf::Color(180, 180, 180));
        option.setPosition(centerX, startY + (i + 1) * dropdownHeight);

        sf::Text optionLabel;
        optionLabel.setFont(font);
        optionLabel.setString(options[i]);
        optionLabel.setCharacterSize(14);
        optionLabel.setFillColor(dropdownText);
        textBounds = optionLabel.getLocalBounds();
        optionLabel.setPosition(
            centerX + 10,
            startY + (i + 1) * dropdownHeight + (dropdownHeight - textBounds.height) / 2.0f - textBounds.top);

        dropdown.options.push_back(option);
        dropdown.optionLabels.push_back(optionLabel);
        dropdown.optionNames.push_back(options[i]);
    }

    return dropdown;
}

#pragma endregion GUI Structs and Classes

#pragma region GUI Helper Functions

// Create a flashing text message
FlashingText createFlashingText(sf::Font &font, const std::string &message, float x, float y, float frequency, sf::Color color)
{
    FlashingText flashingText;
    
    // Split long messages into multiple lines
    std::vector<std::string> lines;
    std::string currentLine = "";
    std::stringstream ss(message);
    std::string word;
    
    const int maxCharsPerLine = 30; // Maximum characters per line
    
    while (ss >> word) {
        // Check if adding this word would make the line too long
        if (currentLine.length() + word.length() + 1 > maxCharsPerLine && !currentLine.empty()) {
            lines.push_back(currentLine);
            currentLine = word;
            
            // Limit to 3 lines maximum
            if (lines.size() >= 3) {
                break;
            }
        } else {
            if (!currentLine.empty()) {
                currentLine += " ";
            }
            currentLine += word;
        }
    }
    
    // Add the last line if it's not empty and we haven't reached the limit
    if (!currentLine.empty() && lines.size() < 3) {
        lines.push_back(currentLine);
    }
    
    // If no lines were created (shouldn't happen), use the original message
    if (lines.empty()) {
        lines.push_back(message);
    }
    
    // Create text objects for each line
    for (size_t i = 0; i < lines.size(); i++) {
        flashingText.textLines.push_back(sf::Text());
        flashingText.textLines.back().setFont(font);
        flashingText.textLines.back().setString(lines[i]);
        flashingText.textLines.back().setCharacterSize(14);
        flashingText.textLines.back().setFillColor(color);
        flashingText.textLines.back().setStyle(sf::Text::Bold);
        
        // Calculate line spacing
        float lineSpacing = 24.0f; // 20px font + 4px spacing
        
        // Center each line horizontally and position vertically
        sf::FloatRect textBounds = flashingText.textLines.back().getLocalBounds();
        float yOffset = (i * lineSpacing) - ((lines.size() - 1) * lineSpacing / 2.0f);
        
        flashingText.textLines.back().setPosition(
            x - textBounds.width / 2.0f - textBounds.left,
            y + yOffset - textBounds.height / 2.0f - textBounds.top);
    }
    
    flashingText.frequency = frequency;
    flashingText.isActive = false;
    flashingText.isVisible = true;
    flashingText.timer = 0.0f;
    
    return flashingText;
}

// Create a multi-line flashing text message positioned on the right side
FlashingText createMultiLineFlashingText(sf::Font &font, const std::vector<std::string> &lines, float x, float y, float frequency, sf::Color color)
{
    FlashingText flashingText;
    
    for (size_t i = 0; i < lines.size(); i++) {
        flashingText.textLines.push_back(sf::Text());
        flashingText.textLines.back().setFont(font);
        flashingText.textLines.back().setString(lines[i]);
        flashingText.textLines.back().setCharacterSize(14);
        flashingText.textLines.back().setFillColor(color);
        flashingText.textLines.back().setStyle(sf::Text::Bold);
        
        // Position each line
        flashingText.textLines.back().setPosition(x, y + i * 18); // 18 pixels between lines
    }
    
    flashingText.frequency = frequency;
    flashingText.isActive = false;
    flashingText.isVisible = true;
    flashingText.timer = 0.0f;
    
    return flashingText;
}

// Update flashing text visibility based on elapsed time
void updateFlashingText(FlashingText &flashingText, float deltaTime)
{
    if (!flashingText.isActive)
        return;
        
    flashingText.timer += deltaTime;
    
    // Toggle visibility based on frequency
    float toggleTime = 1.0f / (2.0f * flashingText.frequency); // Time for one half-cycle (on or off)
    if (flashingText.timer >= toggleTime)
    {
        flashingText.isVisible = !flashingText.isVisible;
        flashingText.timer = 0.0f;
    }
}

#pragma endregion GUI Helper Functions

#pragma region Drawing Functions

void drawBoard(sf::RenderWindow &window)
{
    for (int row = 0; row < BOARD_SIZE; row++)
    {
        for (int col = 0; col < BOARD_SIZE; col++)
        {
            sf::RectangleShape square(sf::Vector2f(SQUARE_SIZE, SQUARE_SIZE));
            square.setPosition(BOARD_OFFSET_X + col * SQUARE_SIZE, BOARD_OFFSET_Y + row * SQUARE_SIZE);
            square.setFillColor((row + col) % 2 == 0 ? cream : brown);
            window.draw(square);
        }
    }
    
    // Draw move indicators
    for (const auto& indicator : moveIndicators) {
        window.draw(indicator);
    }
}

std::vector<sf::Sprite> makePieces(const sf::Texture &texture, std::string fen)
{
    std::vector<sf::Sprite> result;
    int row = 0, col = 0;
    sf::Sprite piece(texture);
    piece.setScale(SQUARE_SIZE / 64.f, SQUARE_SIZE / 64.f);

    // First, initialize the entire board with empty squares
    for (int r = 0; r < 8; r++) {
        for (int c = 0; c < 8; c++) {
            board[r][c] = '-';
        }
    }

    for (char c : fen)
    {
        if (isdigit(c))
        {
            // For digits, we need to mark those squares as empty and advance the column
            int emptySquares = c - '0';
            for (int i = 0; i < emptySquares; i++) {
                if (col < 8) {
                    board[row][col] = '-';
                    col++;
                }
            }
            continue;
        }
        if (c == '/')
        {
            row++;
            col = 0;
            continue;
        }

        int x = 0, y = isupper(c) ? 64 : 0;
        switch (tolower(c))
        {
        case 'k':
            x = 0;
            break;
        case 'q':
            x = 64;
            break;
        case 'r':
            x = 128;
            break;
        case 'n':
            x = 192;
            break;
        case 'b':
            x = 256;
            break;
        case 'p':
            x = 320;
            break;
        }

        piece.setTextureRect(sf::IntRect(x, y, 64, 64));
        piece.setPosition(BOARD_OFFSET_X + col * SQUARE_SIZE, BOARD_OFFSET_Y + row * SQUARE_SIZE);
        result.push_back(piece);
        board[row][col] = c;
        col++;
    }
    
    // Debug: Print the board state
    std::cout << "Board state after FEN parsing:" << std::endl;
    for (int r = 0; r < 8; r++) {
        for (int c = 0; c < 8; c++) {
            std::cout << board[r][c];
        }
        std::cout << std::endl;
    }
    
    return result;
}

void drawTitle(sf::RenderWindow &window, sf::Font &font, GUIState currentState)
{
    sf::Text title;
    title.setFont(font);
    title.setString("Pawn Stars");
    title.setCharacterSize(22);
    title.setFillColor(sf::Color(50, 50, 50));
    title.setStyle(sf::Text::Bold);
    sf::FloatRect textBounds = title.getLocalBounds();
    
    if (currentState == STARTUP_SCREEN) {
        // Center top for startup screen
        title.setPosition((window.getSize().x - textBounds.width) / 2.0f - textBounds.left, 20);
    } else {
        // Keep current position for game screen
        title.setPosition(130, 390);
    }
    
    window.draw(title);
}

void drawStatus(sf::RenderWindow &window, sf::Font &font, const std::string &status)
{
    sf::Text statusText;
    statusText.setFont(font);
    statusText.setString(status);
    statusText.setCharacterSize(14);
    statusText.setFillColor(sf::Color(80, 80, 80));
    int rightPanelX = BOARD_OFFSET_X + BOARD_SIZE * SQUARE_SIZE + 120;
    statusText.setPosition(rightPanelX, BOARD_OFFSET_Y + 300);
    window.draw(statusText);
}

#pragma endregion Drawing Functions

#pragma region ROS2 Topic Subs and Services
class ChessSubscriber : public rclcpp::Node
{
public:
    ChessSubscriber() : Node("chessGUI")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "fen_string", 10, std::bind(&ChessSubscriber::moveCallback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::String>("chess_control", 10);
        
        // Publisher for chess moves (move strings)
        chess_moves_publisher_ = this->create_publisher<std_msgs::msg::String>("/chess_moves", 10);

        chess_moves_pub_stockfish_ = this->create_publisher<std_msgs::msg::String>("/player_move", 10);

        reset_client_ = this->create_client<std_srvs::srv::Trigger>("/reset_chessboard");

        button_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "button_state", 10, [this](std_msgs::msg::Bool::SharedPtr msg)
            {
                buttonState = msg->data;
                RCLCPP_INFO(this->get_logger(), "Button state: %s", buttonState ? "ON" : "OFF"); });

        status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/status", 10, [this](std_msgs::msg::String::SharedPtr msg)
            {
                if (msg->data.size() >= 4) {
                    status1 = (msg->data[0] == '1');  // Board out of range
                    status2 = (msg->data[1] == '1');  // Status 2
                    status3 = (msg->data[2] == '1');  // E-stop flag
                    status4 = (msg->data[3] == '1');  // Player turn
                    RCLCPP_INFO(this->get_logger(), "Status received: %c %c %c %c", msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Status string too short: '%s'", msg->data.c_str());
                }
            });  
    }

    void callResetService()
    {
        if (!reset_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Reset service not available.");
            return;
        }
    
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = reset_client_->async_send_request(request,
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture response) {
                auto result = response.get();
                if (result->success) {
                    RCLCPP_INFO(this->get_logger(), "Reset successful: %s", result->message.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "Reset failed: %s", result->message.c_str());
                }
            });
    }

    std::string trimFenToBoard(const std::string& fenString) {
        // Find the position of the first whitespace
        size_t spacePos = fenString.find(' ');
        
        // If a space was found, return everything before it
        if (spacePos != std::string::npos) {
            return fenString.substr(0, spacePos);
        }
        
        // If no space was found, return the entire string
        return fenString;
    }

    void publishControl(const std::string &command)
    {
        auto msg = std_msgs::msg::String();
        msg.data = command;
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published control: %s", command.c_str());
    }

    std::string getLastMove() const { return lastMove; }
    bool getButtonState() const { return buttonState; }
    bool getStatus1() const { return status1; }
    bool getStatus2() const { return status2; }
    bool getStatus3() const { return status3; }
    bool getStatus4() const { return status4; }
    // Legacy getters for backward compatibility
    bool getBoardOutOfRange() const { return status1; }
    bool getPlayerTurn() const { return status4; }
    bool getEstopFlag() const { return status3; }
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr chess_moves_pub_stockfish_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr chess_moves_publisher_;
private:
    void moveCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string fullFen = msg->data;
        std::string boardOnly = trimFenToBoard(fullFen);
        
        RCLCPP_INFO(this->get_logger(), "Received move: %s", fullFen.c_str());
        RCLCPP_INFO(this->get_logger(), "Board state only: %s", boardOnly.c_str());
        
        // Store the trimmed version
        lastMove = boardOnly;
    }

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_client_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr button_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr board_oor_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr player_turn_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_flag_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
    char temp1, temp2, temp3, temp4;
    std::string lastMove;
    bool buttonState = false;
    bool status1 = false;
    bool status2 = false;
    bool status3 = false;
    bool status4 = false;
};
#pragma endregion ROS
std::string createMoveString(int fromRow, int fromCol, int toRow, int toCol, char piece, char pieceCap, bool isCapture, bool isCastling, bool isPromotion, std::shared_ptr<ChessSubscriber> ros2node);

#pragma region Main Function
// Main function

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChessSubscriber>();

    std::string lastFenRendered;
    std::string fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR";
    std::string status = "Ready to play";
    
    // Track the current GUI state
    GUIState currentState = STARTUP_SCREEN;
    
    // Store selected difficulty
    std::string selectedDifficulty = "Easy";
    
    // Robot Only Mode state
    bool robotOnlyMode = false;

    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Chess Visualizer", sf::Style::Fullscreen);
    window.setVerticalSyncEnabled(true);

    sf::Texture texture;
    if (!texture.loadFromFile(chessPiecesPath))
    {
        std::cerr << "Failed to load chess pieces texture!" << std::endl;
        return 1;
    }

    std::vector<sf::Sprite> pieces = makePieces(texture, fen);
    sf::Font font;
    if (!font.loadFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"))
    {
        std::cerr << "Font loading failed!" << std::endl;
        return 1;
    }

    // Create startup screen elements
    Dropdown difficultyDropdown = createDifficultyDropdown(font);
    
    // Create start game button (centered and below dropdown)
    int startButtonWidth = 200;
    int startButtonHeight = 50;
    int startButtonX = (WINDOW_WIDTH - startButtonWidth) / 2 - 180;
    int startButtonY = WINDOW_HEIGHT / 2 - 5;
    Button startButton = createButton(font, "START GAME", startButtonX, startButtonY, 
                                     startButtonWidth, startButtonHeight, 
                                     startButtonGreen, startButtonHover);
    
    // Create Robot Only Mode button (next to start button)
    int robotModeButtonWidth = 200;
    int robotModeButtonHeight = 50;
    int robotModeButtonX = startButtonX; // 20px gap
    int robotModeButtonY = startButtonY+ 70;
    Button robotModeButton = createButton(font, "MANUAL ONLY: On", robotModeButtonX, robotModeButtonY,
                                         robotModeButtonWidth, robotModeButtonHeight,
                                         buttonBlue, startButtonHover); // Gray colors initially
    
    // Create game screen elements
    std::vector<Button> gameButtons = createGameButtons(font);
    
    // Debug output to confirm buttons
    std::cout << "Created " << gameButtons.size() << " game buttons" << std::endl;
    
    // Create a dedicated reset button for the game screen
    Button resetButton = createButton(font, "BACK TO MENU", 
                                    BOARD_OFFSET_X + BOARD_SIZE * SQUARE_SIZE + 120, 
                                    BOARD_OFFSET_Y + 320, 
                                    160, 50, 
                                    sf::Color(220, 60, 60), sf::Color(255, 100, 100));
                                    
    // Create flashing text notifications
    FlashingText boardOutOfRangeText = createFlashingText(
        font, 
        "Board out of range, please reposition", 
        WINDOW_WIDTH / 2 + 30, 
        WINDOW_HEIGHT/ 2 + 80, 
        1.0f, // 2 flashes per second
        warningRed
    );

    FlashingText playerTurnText = createFlashingText(
        font, 
        "Player's turn, please make your move, then press the Button.", 
        WINDOW_WIDTH / 2, 
        BOARD_OFFSET_Y + BOARD_SIZE * SQUARE_SIZE + 80, 
        0.5f, // 1.5 flashes per second
        sf::Color(0, 0, 0) // Blue color for player turn notification
    );
    
    FlashingText eStopEngaged = createFlashingText(
        font, 
        "Software E-Stop Engaged, Please press the button.", 
        WINDOW_WIDTH / 2, 
        BOARD_OFFSET_Y + BOARD_SIZE * SQUARE_SIZE + 80, 
        1.0f, // 1.5 flashes per second
        warningRed 
    );

        FlashingText garbageCanRange = createFlashingText(
        font, 
        "Move the bin thanks", 
        WINDOW_WIDTH / 2, 
        BOARD_OFFSET_Y + BOARD_SIZE * SQUARE_SIZE + 80, 
        1.0f, // 1.5 flashes per second
        warningRed 
    );
    
    
    // Clock for delta time calculation
    sf::Clock clock;
    
    while (window.isOpen() && rclcpp::ok())
    {
        rclcpp::spin_some(node);
        
        // Calculate delta time for flashing text updates
        float deltaTime = clock.restart().asSeconds();
        
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
                
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Escape)
                window.close();
                
            if (event.type == sf::Event::MouseMoved)
            {
                sf::Vector2f mousePos(event.mouseMove.x, event.mouseMove.y);
                
                if (currentState == STARTUP_SCREEN) {
                    // Handle start button hover effect
                    bool wasStartHovered = startButton.isHovered;
                    startButton.isHovered = startButton.shape.getGlobalBounds().contains(mousePos);
                    if (startButton.isHovered != wasStartHovered) {
                        startButton.shape.setFillColor(startButton.isHovered ? startButtonHover : startButtonGreen);
                    }
                    
                    // Handle Robot Only Mode button hover effect
                    bool wasRobotModeHovered = robotModeButton.isHovered;
                    robotModeButton.isHovered = robotModeButton.shape.getGlobalBounds().contains(mousePos);
                    if (robotModeButton.isHovered != wasRobotModeHovered) {
                        sf::Color baseColor = robotOnlyMode ? startButtonGreen : buttonBlue;
                        sf::Color hoverColor = robotOnlyMode ? startButtonHover : buttonHover;
                        robotModeButton.shape.setFillColor(robotModeButton.isHovered ? hoverColor : baseColor);
                    }
                } else if (currentState == GAME_SCREEN) {
                    // Handle game buttons hover effect
                    for (auto &button : gameButtons) {
                        bool wasHovered = button.isHovered;
                        button.isHovered = button.shape.getGlobalBounds().contains(mousePos);
                        if (button.isHovered != wasHovered) {
                            button.shape.setFillColor(button.isHovered ? buttonHover : buttonBlue);
                        }
                    }
                    
                    // Handle reset button hover effect
                    bool wasResetHovered = resetButton.isHovered;
                    resetButton.isHovered = resetButton.shape.getGlobalBounds().contains(mousePos);
                    if (resetButton.isHovered != wasResetHovered) {
                        resetButton.shape.setFillColor(resetButton.isHovered ? sf::Color(255, 100, 100) : sf::Color(220, 60, 60));
                    }
                }
            }
            
            if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left)
            {
                sf::Vector2f mousePos(event.mouseButton.x, event.mouseButton.y);
                
                if (currentState == STARTUP_SCREEN) {
                    // Handle dropdown interaction
                    if (difficultyDropdown.mainButton.getGlobalBounds().contains(mousePos)) {
                        difficultyDropdown.isOpen = !difficultyDropdown.isOpen;
                    } else if (difficultyDropdown.isOpen) {
                        for (int i = 0; i < difficultyDropdown.options.size(); i++) {
                            if (difficultyDropdown.options[i].getGlobalBounds().contains(mousePos)) {
                                difficultyDropdown.selectedIndex = i;
                                selectedDifficulty = difficultyDropdown.optionNames[i];
                                difficultyDropdown.mainLabel.setString("Difficulty: " + selectedDifficulty);
                                difficultyDropdown.isOpen = false;
                                
                                sf::FloatRect textBounds = difficultyDropdown.mainLabel.getLocalBounds();
                                float x = difficultyDropdown.mainButton.getPosition().x + 10;
                                float y = difficultyDropdown.mainButton.getPosition().y +
                                          (difficultyDropdown.mainButton.getSize().y - textBounds.height) / 2.0f - textBounds.top;
                                difficultyDropdown.mainLabel.setPosition(x, y);
                            }
                        }
                    }
                    
                    // Check for start button click
                    if (startButton.shape.getGlobalBounds().contains(mousePos)) {
                        std::cout << "Start button clicked!" << std::endl;
                        currentState = GAME_SCREEN;
                        // Send difficulty to the game system
                       // node->publishControl("DIFFICULTY_" + selectedDifficulty);
                        status = "Game started with " + selectedDifficulty + " difficulty";
                        // Reset the board via ROS service
                        node->callResetService();
                    }
                    
                    // Check for Robot Only Mode button click
                    if (robotModeButton.shape.getGlobalBounds().contains(mousePos)) {
                        robotOnlyMode = !robotOnlyMode;
                        std::cout << "Manual Only Mode: " << (robotOnlyMode ? "Off" : "On") << std::endl;
                        
                        // Update button text and colors
                        robotModeButton.label.setString(robotOnlyMode ? "MANUAL ONLY: Off" : "MANUAL ONLY: On");
                        sf::Color baseColor = robotOnlyMode ? startButtonGreen : buttonBlue;
                        robotModeButton.shape.setFillColor(baseColor);
                        
                        // Re-center the text
                        sf::FloatRect textBounds = robotModeButton.label.getLocalBounds();
                        robotModeButton.label.setPosition(
                            robotModeButtonX + (robotModeButtonWidth - textBounds.width) / 2.0f - textBounds.left,
                            robotModeButtonY + (robotModeButtonHeight - textBounds.height) / 2.0f - textBounds.top);
                    }
                } else if (currentState == GAME_SCREEN) {
                    // Check if click is within the chess board
                    if (mousePos.x >= BOARD_OFFSET_X && mousePos.x < BOARD_OFFSET_X + BOARD_SIZE * SQUARE_SIZE &&
                        mousePos.y >= BOARD_OFFSET_Y && mousePos.y < BOARD_OFFSET_Y + BOARD_SIZE * SQUARE_SIZE) {
                        
                        int col = (mousePos.x - BOARD_OFFSET_X) / SQUARE_SIZE;
                        int row = (mousePos.y - BOARD_OFFSET_Y) / SQUARE_SIZE;
                        
                        std::cout << "\nBoard clicked!" << std::endl;
                        
                        // Check if we have a selected piece and if this click is on a legal move
                        if (selectedPiece.x != -1 && selectedPiece.y != -1) {
                            // Check if the clicked position is in possible moves
                            bool isLegalMove = false;
                            for (const auto& move : possibleMoves) {
                                if (move[0] == row && move[1] == col) {
                                    isLegalMove = true;
                                    break;
                                }
                            }
                            
                            if (isLegalMove) {
                                // Check if Robot Only Mode is enabled
                                if (robotOnlyMode) {
                                    // Get the piece before executing the move
                                    char piece = board[selectedPiece.x][selectedPiece.y];
                                    
                                    // Execute the move
                                    MoveInfo moveInfo = executeMove(selectedPiece.x, selectedPiece.y, row, col);
                                    
                                    // Create move string
                                    std::string moveString = createMoveString(selectedPiece.x, selectedPiece.y, row, col, piece, moveInfo.captured, moveInfo.isCapture, moveInfo.isCastling, moveInfo.isPromotion, node);
                                    
                                    // Output move information to terminal
                                    std::cout << "=========================" << std::endl;
                                    std::cout << "MOVE EXECUTED!" << std::endl;
                                    std::cout << "From: " << coordsToChessNotation(selectedPiece.x, selectedPiece.y) << std::endl;
                                    std::cout << "To: " << coordsToChessNotation(row, col) << std::endl;
                                    std::cout << "Piece: " << piece << std::endl;
                                    std::cout << "Move String: " << moveString << std::endl;
                                    std::cout << "=========================" << std::endl;
                                    
                                    // Publish the move string
//                                    node->publishChessMove(moveString);
                                    
                                    // Clear selection
                                    selectedPiece = sf::Vector2i(-1, -1);
                                    moveIndicators.clear();
                                    possibleMoves.clear();
                                    
                                    status = "Move executed! Check terminal for move string.";
                                } else {
                                    // Robot Only Mode is off, just show feedback
                                    std::cout << "Move not executed - Robot Only Mode is OFF" << std::endl;
                                    std::cout << "Legal move: row=" << selectedPiece.x << ", col=" << selectedPiece.y 
                                              << " -> row=" << row << ", col=" << col << std::endl;
                                    
                                    // Clear selection but don't execute move
                                    selectedPiece = sf::Vector2i(-1, -1);
                                    moveIndicators.clear();
                                    possibleMoves.clear();
                                    
                                    status = "Robot Only Mode OFF - Enable to execute moves";
                                }
                            } else {
                                // Not a legal move, select new piece at this position
                                updateMoveIndicators(row, col);
                            }
                        } else {
                            // No piece selected, select piece at this position
                            updateMoveIndicators(row, col);
                        }
                    }
                    
                    // Check for game button clicks
                    for (const auto &button : gameButtons) {
                        if (button.shape.getGlobalBounds().contains(mousePos)) {
                            status = "Command: " + button.name;
                          //  node->publishControl(button.name);
                        }
                    }
                    
                    // Check for reset button click to go back to menu
                    if (resetButton.shape.getGlobalBounds().contains(mousePos)) {
                        std::cout << "Back to menu button clicked!" << std::endl;
                        currentState = STARTUP_SCREEN;
                        status = "Ready to play";
                    }
                }
            }
        }

        // Update flashing text state based on ROS topic data
        if (currentState == GAME_SCREEN) {
            // // Update board out of range warning
            // boardOutOfRangeText.isActive = node->getBoardOutOfRange();
            // updateFlashingText(boardOutOfRangeText, deltaTime);
            
            // // Update player turn notification
            // playerTurnText.isActive = node->getPlayerTurn();
            // updateFlashingText(playerTurnText, deltaTime);

            // eStopEngaged.isActive = node->getEstopFlag();
            // updateFlashingText(eStopEngaged, deltaTime);

        }

        // Update game board if needed
        if (currentState == GAME_SCREEN) {
            std::string currentFen = node->getLastMove();
            if (!currentFen.empty() && currentFen != lastFenRendered) {
                pieces = makePieces(texture, currentFen);  // Rebuild piece sprites from FEN
                lastFenRendered = currentFen;
                status = "Updated board from FEN";
                std::cout << "Board updated with new FEN: " << currentFen << std::endl;
            }
        }

        window.clear(sf::Color(180, 180, 180));
        drawTitle(window, font, currentState);
        
        if (currentState == STARTUP_SCREEN) {
            // Draw startup screen elements
            
            // Draw instruction text
            sf::Text instructionText;
            instructionText.setFont(font);
            instructionText.setString("Select difficulty level and press Start Game");
            instructionText.setCharacterSize(18);
            instructionText.setFillColor(sf::Color(50, 50, 50));
            sf::FloatRect textBounds = instructionText.getLocalBounds();
            instructionText.setPosition(
                (WINDOW_WIDTH - textBounds.width) / 2.0f - textBounds.left,
                WINDOW_HEIGHT / 2 - 100);
            window.draw(instructionText);
            
            // Draw dropdown
            window.draw(difficultyDropdown.mainButton);
            window.draw(difficultyDropdown.mainLabel);
            if (difficultyDropdown.isOpen) {
                for (int i = 0; i < difficultyDropdown.options.size(); i++) {
                    window.draw(difficultyDropdown.options[i]);
                    window.draw(difficultyDropdown.optionLabels[i]);
                }
            }
            
            // Draw start button
            window.draw(startButton.shape);
            window.draw(startButton.label);
            
            // Draw Robot Only Mode button
            window.draw(robotModeButton.shape);
            window.draw(robotModeButton.label);
            
        } else if (currentState == GAME_SCREEN) {
            // Draw game screen elements
            drawBoard(window);
            for (const auto &piece : pieces)
                window.draw(piece);
                
            for (const auto &button : gameButtons) {
                window.draw(button.shape);
                window.draw(button.label);
            }
            
            //drawStatus(window, font, status);
            
            // Draw flashing texts if they're active and visible
            if (boardOutOfRangeText.isActive && boardOutOfRangeText.isVisible) {
                for (const auto& text : boardOutOfRangeText.textLines) {
                    window.draw(text);
                }
            }
            
            if (playerTurnText.isActive && playerTurnText.isVisible) {
                for (const auto& text : playerTurnText.textLines) {
                    window.draw(text);
                }
            }
            
            if (eStopEngaged.isActive && eStopEngaged.isVisible) {
                for (const auto& text : eStopEngaged.textLines) {
                    window.draw(text);
                }
            }
            
            // Draw the dedicated reset/back button
            window.draw(resetButton.shape);
            window.draw(resetButton.label);
            
            // Indicator light
            sf::CircleShape indicator(20); // The parameter is the radius (half of the previous width/height of 40)
            indicator.setPosition(WINDOW_WIDTH - 50, 10);
            indicator.setFillColor(node->getButtonState() ? buttonBlue : sf::Color::Red);
            indicator.setOutlineColor(sf::Color::Black);
            indicator.setOutlineThickness(1);
            window.draw(indicator);

            sf::Text indicatorLabel;
            indicatorLabel.setFont(font);
            indicatorLabel.setCharacterSize(16);
            indicatorLabel.setFillColor(sf::Color(50, 50, 50));

            if (node->getButtonState()) {
                // Robot is UNLOCKED
                indicatorLabel.setString(" Robot unlocked: ");
                indicatorLabel.setPosition(WINDOW_WIDTH - 220, 22);
            } else {
                indicatorLabel.setString(" Robot locked: ");
                indicatorLabel.setPosition(WINDOW_WIDTH - 200, 22);
            }

            window.draw(indicatorLabel);
            
            // Status indicators
            std::vector<std::string> statusLabels = {" Board Range", "       Status 2", ""};  // Empty string for Player Turn - will be set dynamically
            std::vector<std::string> errorMessages = {"Reposition the board", "Check system status", "Make your move"};
            std::vector<bool> statusValues = {node->getStatus1(), node->getStatus2(), node->getStatus4()};
            
            for (int i = 0; i < 3; i++) {  // Changed from 4 to 3
                // Status indicator circle - increased size to match robot indicator
                sf::CircleShape statusIndicator(20);  // Increased from 15 to 20 to match robot button
                statusIndicator.setPosition(WINDOW_WIDTH - 50, 70 + i * 60);  // Increased gap from 50 to 60, adjusted X position
                
                // Special handling for Player Turn indicator (index 2)
                if (i == 2) {
                    // Use black/white colors for player turn instead of red/green
                    statusIndicator.setFillColor(statusValues[i] ? sf::Color::White : sf::Color::Black);
                    statusIndicator.setOutlineColor(sf::Color::Black);
                    statusIndicator.setOutlineThickness(2);  // Thicker outline for better visibility with white
                } else {
                    statusIndicator.setFillColor(statusValues[i] ? sf::Color::Red : sf::Color::Green);
                    statusIndicator.setOutlineColor(sf::Color::Black);
                    statusIndicator.setOutlineThickness(1);
                }
                window.draw(statusIndicator);
                
                // Status label
                sf::Text statusLabel;
                statusLabel.setFont(font);
                
                // Dynamic label for Player Turn
                if (i == 2) {
                    statusLabel.setString(statusValues[i] ? "    White Turn:" : "    Black Turn:");
                } else {
                    statusLabel.setString(statusLabels[i] + ":");
                }
                
                statusLabel.setCharacterSize(16);  // Increased from 12 to 16 to match robot indicator
                statusLabel.setFillColor(sf::Color(50, 50, 50));
                statusLabel.setPosition(WINDOW_WIDTH - 195, 82 + i * 60);  // Moved further left from -160 to -220
                window.draw(statusLabel);
                
                // Error message (only show when status is red/true)
                if (statusValues[i]) {
                    sf::Text errorText;
                    errorText.setFont(font);
                    errorText.setString(errorMessages[i]);
                    errorText.setCharacterSize(12);  // Kept error text smaller for readability
                    errorText.setFillColor(sf::Color::Red);
                    errorText.setStyle(sf::Text::Italic);
                    errorText.setPosition(WINDOW_WIDTH - 220, 100 + i * 60);  // Increased gap from 86 to 92 (6 more pixels)
                    window.draw(errorText);
                }
            }
            
            // Display current difficulty in game screen
            sf::Text difficultyText;
            difficultyText.setFont(font);
            difficultyText.setString("Difficulty: " + selectedDifficulty);
            difficultyText.setCharacterSize(14);
            difficultyText.setFillColor(sf::Color(50, 50, 50));
            difficultyText.setPosition(BOARD_OFFSET_X + BOARD_SIZE * SQUARE_SIZE + 40, BOARD_OFFSET_Y + 30);
            window.draw(difficultyText);
            
            // Display Robot Only Mode status
            sf::Text robotModeText;
            robotModeText.setFont(font);
            robotModeText.setString("Manual Only Mode: " + std::string(robotOnlyMode ? "Off" : "On"));
            robotModeText.setCharacterSize(14);
            robotModeText.setFillColor(robotOnlyMode ? sf::Color((180, 0, 0)) : sf::Color(0, 180, 0));
            robotModeText.setPosition(BOARD_OFFSET_X + BOARD_SIZE * SQUARE_SIZE + 10, BOARD_OFFSET_Y + 4);
            window.draw(robotModeText);
        }
        
        window.display();
    }

    rclcpp::shutdown();
    return 0;
}

#pragma endregion Main Function

#pragma region Utility Functions

// Add this function definition before main
void updateMoveIndicators(int row, int col) {
    moveIndicators.clear();
    std::cout << "Clicked position: row=" << row << ", col=" << col << std::endl;
    
    // If no piece is selected or invalid position
    if (row < 0 || col < 0 || row >= BOARD_SIZE || col >= BOARD_SIZE) {
        selectedPiece = sf::Vector2i(-1, -1);
        std::cout << "Invalid position" << std::endl;
        return;
    }
    
    // Check if it's a white piece
    char piece = board[row][col];
    std::cout << "Selected piece: " << piece << std::endl;
    
    if (!std::isupper(piece)) {
        selectedPiece = sf::Vector2i(-1, -1);
        std::cout << "Not a white piece" << std::endl;
        return;
    }
    
    selectedPiece = sf::Vector2i(row, col);
    possibleMoves = getPossibleMoves(row, col);
    std::cout << "Number of possible moves: " << possibleMoves.size() << std::endl;
    
    // Create visual indicators for possible moves
    for (const auto& move : possibleMoves) {
        sf::CircleShape indicator(SQUARE_SIZE / 3.0f); // Made indicators larger
        indicator.setPosition(
            BOARD_OFFSET_X + move[1] * SQUARE_SIZE + SQUARE_SIZE/2 - indicator.getRadius(),
            BOARD_OFFSET_Y + move[0] * SQUARE_SIZE + SQUARE_SIZE/2 - indicator.getRadius()
        );
        indicator.setFillColor(sf::Color(0, 255, 0, 128)); // Changed to green with more opacity
        moveIndicators.push_back(indicator);
        std::cout << "Added move indicator at row=" << move[0] << ", col=" << move[1] << std::endl;
    }
}

// Function to convert row/col to chess notation (e.g., row=0, col=4 -> "e8")
std::string coordsToChessNotation(int row, int col) {
    if (row < 0 || row > 7 || col < 0 || col > 7) return "";
    
    char file = 'a' + col;  // columns a-h
    char rank = '8' - row;  // rows 8-1 (row 0 = rank 8, row 7 = rank 1)
    
    return std::string(1, file) + std::string(1, rank);
}

// Function to create move string in format: moveplayed + moveType + piecePlayed
std::string createMoveString(int fromRow, int fromCol, int toRow, int toCol, char piece, char pieceCap, bool isCapture, bool isCastling, bool isPromotion, std::shared_ptr<ChessSubscriber> ros2node) {
    std::string moveStr = "";
    
    // 1. Move played (from + to coordinates)
    std::string fromSquare = coordsToChessNotation(fromRow, fromCol);
    std::string toSquare = coordsToChessNotation(toRow, toCol);
    moveStr += fromSquare + toSquare;
    auto msg = std_msgs::msg::String();
    auto msgToControlNode = std_msgs::msg::String();
    std::string toStockFishNode;
    if (isCapture) {
	toStockFishNode += std::to_string(toRow) + std::to_string(toCol) + "99";
    }

    toStockFishNode += std::to_string(fromRow) + std::to_string(fromCol) + std::to_string(toRow) + std::to_string(toCol);
    // 2. Handle castling special case - include both king and rook moves
    if (isCastling) {
        // Determine rook movement based on king's destination
        std::string rookFromSquare, rookToSquare;
        if (toCol == 6) { // Kingside castling (king goes to g-file)
            rookFromSquare = coordsToChessNotation(fromRow, 7); // Rook from h-file
            rookToSquare = coordsToChessNotation(fromRow, 5);   // Rook to f-file
            toStockFishNode += std::to_string(fromRow) + "7" + std::to_string(fromRow) + "5";
        } else if (toCol == 2) { // Queenside castling (king goes to c-file)
            rookFromSquare = coordsToChessNotation(fromRow, 0); // Rook from a-file
            rookToSquare = coordsToChessNotation(fromRow, 3);   // Rook to d-file
	    toStockFishNode += std::to_string(fromRow) + "0" + std::to_string(fromRow) + "3";
        }

        RCLCPP_INFO(ros2node->get_logger(), "Published extra castling rook move to stockfish node: %s", toStockFishNode.c_str());
        // Add rook movement to the string
        //moveStr += rookFromSquare + rookToSquare;
        msg.data = toStockFishNode;
        ros2node->chess_moves_pub_stockfish_->publish(msg);
        // Add move type and pieces (king + rook)
        moveStr += "n";  // castling is essentially 2 normal moves
        moveStr += std::tolower(piece);  // king piece (k)
        msgToControlNode.data = moveStr;
        ros2node->chess_moves_publisher_->publish(msgToControlNode);
        moveStr = rookFromSquare + rookToSquare + "nr";
        msgToControlNode.data = moveStr;
        ros2node->chess_moves_publisher_->publish(msgToControlNode);
        return moveStr;
    }
    msg.data = toStockFishNode;
    ros2node->chess_moves_pub_stockfish_->publish(msg);
    RCLCPP_INFO(ros2node->get_logger(), "Published move to stockfish node: %s", toStockFishNode.c_str());
    // 3. Move type (for non-castling moves)
    if (isPromotion) {
        moveStr += "p";  // promotion
    } else if (isCapture) {
        moveStr += "x";  // capture
    } else {
        moveStr += "n";  // normal move
    }
    
    // 4. Piece played (convert to lowercase)
    // For promotion, we use 'q' since we always promote to queen
    if (isPromotion) {
        moveStr += "q";  // Always promote to queen
    } else {
        char pieceLower = std::tolower(piece);
        moveStr += pieceLower;
	if (isCapture) moveStr += pieceCap;
    }
    msgToControlNode.data = moveStr;
    ros2node->chess_moves_publisher_->publish(msgToControlNode);
    return moveStr;
}

#pragma endregion Utility Functions

#pragma region TODO Notes

//TODO 
// Assign GUI elements to the Error flags currently temp1, temp2, temp3, temp4

#pragma endregion TODO Notes
