#include <iostream>
#include <cstdio>
#include <vector>
#include <cstring>
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

void drawBoard(sf::RenderWindow &window) {
    for (int row = 0; row < BOARD_SIZE; row++) {
        for (int col = 0; col < BOARD_SIZE; col++) {
            sf::RectangleShape square(sf::Vector2f(SQUARE_SIZE, SQUARE_SIZE));
            square.setPosition(col * SQUARE_SIZE, row * SQUARE_SIZE);

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
                    for (int i = 0; i < pieces.size(); i++) {
                        if (pieces[i].getGlobalBounds().contains(mousePosF)) {
                            unsigned char clickedPiece = board[(int(pieces[i].getPosition().y)-8)/SQUARE_SIZE][(int(pieces[i].getPosition().x)-8)/SQUARE_SIZE];
                            std::cout << clickedPiece << '\n';
                            break;
                        }
                    }
                }
            }
        }

        window.clear();
        drawBoard(window);
        for (int i = 0; i < pieces.size(); i++) {
            window.draw(pieces[i]);
        }
        window.display();
    }
    return 0;
}
