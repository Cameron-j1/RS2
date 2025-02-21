#include <iostream>
#include <cstdio>
#include <vector>
#include <cstring>
#include <algorithm>
#include <cctype>
#include <unordered_map>
#include <SFML/Graphics.hpp>

const int BOARD_SIZE = 8;
const int SQUARE_SIZE = 80;  // Adjust as needed

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

int main() {
    sf::RenderWindow window(sf::VideoMode(BOARD_SIZE * SQUARE_SIZE, BOARD_SIZE * SQUARE_SIZE), "Chess GUI", sf::Style::Titlebar | sf::Style::Close);
    sf::Texture texture;
    texture.loadFromFile("pieces.png");
    std::string fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR";

    window.setVerticalSyncEnabled(false);
    std::vector<std::string> pieceName;
    std::vector<sf::Sprite> pieces = makePieces(texture, fen);
    int lastClickedPiece = -1;
    int curRow = 0, curCol = 0;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();

            if (event.type == sf::Event::MouseButtonPressed) {
                if (event.mouseButton.button == sf::Mouse::Left) {
                    sf::Vector2i mousePos = sf::Mouse::getPosition(window);
                    // Convert to float for bounds checking
                    sf::Vector2f mousePosF(static_cast<float>(mousePos.x), static_cast<float>(mousePos.y));
                    bool foundPiece = false;

                    // First mouse click, to select the pieces and display possible moves for that piece
                    for (int i = 0; i < pieces.size(); i++) {
                        if (pieces[i].getGlobalBounds().contains(mousePosF)) {
                            if (lastClickedPiece == -1) {
                                curRow = (int(pieces[i].getPosition().y)-8)/SQUARE_SIZE;
                                curCol = (int(pieces[i].getPosition().x)-8)/SQUARE_SIZE;
                                foundPiece = true;
                                lastClickedPiece = i;
                                auto moves = getPossibleMoves(curRow, curCol);
                                isPotentialMove.assign(isPotentialMove.size() - 1, false);

                                for (const auto& move : moves) {
                                    isPotentialMove[move[0]*8 + move[1]] = true;
                                }
                            }

                            else if (lastClickedPiece != -1) {
                                pieces[i].setScale(0.0f, 0.0f);
                            }
                            break;
                        }
                    }
                    
                    // Second click to confirm next move
                    if (!foundPiece && lastClickedPiece != -1) {
                        for (int i = 0; i < potentialMove.size(); i++) {
                            if (potentialMove[i].getGlobalBounds().contains(mousePosF) && isPotentialMove[i] == true) {
                                int nextRow = (i/8) * SQUARE_SIZE + 8, nextCol = (i%8) * SQUARE_SIZE + 8;
                                pieces[lastClickedPiece].setPosition(nextCol, nextRow);
                                board[i/8][i%8] = board[curRow][curCol];
                                board[curRow][curCol] = '-';
                                // Move the corresponding rook if a castling move is detected
                                if (isCastling[i] == true) {
                                    // std::cout << "Castle \n";
                                    int rookNextCol = i % 8 == 2 ? 3 : 5, rookCurCol = i % 8 == 2 ? 0 : 7;
                                    for (int y = 0; y < pieces.size(); y++) {
                                        if ((int(pieces[y].getPosition().y)-8)/SQUARE_SIZE == curRow && (int(pieces[y].getPosition().x)-8)/SQUARE_SIZE == rookCurCol) {
                                            pieces[y].setPosition(rookNextCol * SQUARE_SIZE + 8, curRow * SQUARE_SIZE + 8);
                                            board[curRow][rookNextCol] = board[curRow][rookCurCol];
                                            board[curRow][rookCurCol] = '-';
                                            break;
                                        }
                                    }
                                }

                                lastClickedPiece = -1;
                                break;
                            }
                        }
                        for (auto& pair : isCastling) {
                            pair.second = false;
                        }
                        isPotentialMove.assign(isPotentialMove.size() - 1, false);
                    }

                    // Print the current state of the chess board
                    // for (int i = 0; i < 8; i++) {
                    //     for (int y = 0; y < 8; y++) {
                    //         std::cout << board[i][y] << ' ';
                    //     }
                    //     std::cout << '\n';
                    // }
                    // std::cout << '\n';
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
    }
    return 0;
}
