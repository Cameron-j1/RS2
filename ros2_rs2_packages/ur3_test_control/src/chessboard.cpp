#include <iostream>
#include <SFML/Graphics.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <vector>
#include <string>
#include <std_srvs/srv/trigger.hpp>

// Configs
const int BOARD_SIZE = 8;
const int SQUARE_SIZE = 45;
const int BOARD_OFFSET_X = 25;
const int BOARD_OFFSET_Y = 40;
const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 460;

// Colors
sf::Color cream(211, 211, 211); // GREY
sf::Color brown(70, 130, 180);  // LIGHT BLUE
sf::Color buttonBlue(70, 130, 180);
sf::Color buttonHover(100, 160, 250);
sf::Color buttonText(255, 255, 255);
sf::Color dropdownBg(230, 230, 230);
sf::Color dropdownText(50, 50, 50);

// Texture path
std::string chessPiecesPath = ament_index_cpp::get_package_share_directory("ur3_test_control") + "/images/pieces.png";

// Button and Dropdown structs
struct Button
{
    sf::RectangleShape shape;
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

std::vector<Button> createButtons(sf::Font &font)
{
    std::vector<Button> buttons;
    std::vector<std::string> labels = {"ON", "OFF", "PAUSE", "RESET"};

    int buttonWidth = 120, buttonHeight = 40;
    int rightPanelX = BOARD_OFFSET_X + BOARD_SIZE * SQUARE_SIZE + 60;

    for (int i = 0; i < labels.size(); i++)
    {
        Button button;
        button.shape.setSize(sf::Vector2f(buttonWidth, buttonHeight));
        button.shape.setFillColor(buttonBlue);
        button.shape.setOutlineThickness(1);
        button.shape.setOutlineColor(sf::Color(40, 100, 150));
        int startX = rightPanelX;
        int startY = BOARD_OFFSET_Y + 20 + i * (buttonHeight + 15);
        button.shape.setPosition(startX, startY);
        button.label.setFont(font);
        button.label.setString(labels[i]);
        button.label.setCharacterSize(16);
        button.label.setFillColor(buttonText);
        sf::FloatRect textBounds = button.label.getLocalBounds();
        button.label.setPosition(
            startX + (buttonWidth - textBounds.width) / 2.0f - textBounds.left,
            startY + (buttonHeight - textBounds.height) / 2.0f - textBounds.top);
        button.name = labels[i];
        buttons.push_back(button);
    }

    return buttons;
}

Dropdown createDifficultyDropdown(sf::Font &font)
{
    Dropdown dropdown;
    std::vector<std::string> options = {"Easy", "Medium", "Hard"};
    int dropdownWidth = 200, dropdownHeight = 40;
    int rightPanelX = BOARD_OFFSET_X + BOARD_SIZE * SQUARE_SIZE + 30;
    int startY = BOARD_OFFSET_Y + 200;

    dropdown.mainButton.setSize(sf::Vector2f(dropdownWidth, dropdownHeight));
    dropdown.mainButton.setFillColor(dropdownBg);
    dropdown.mainButton.setOutlineThickness(1);
    dropdown.mainButton.setOutlineColor(sf::Color(180, 180, 180));
    dropdown.mainButton.setPosition(rightPanelX, startY);
    dropdown.mainLabel.setFont(font);
    dropdown.mainLabel.setString("Difficulty: " + options[0]);
    dropdown.mainLabel.setCharacterSize(14);
    dropdown.mainLabel.setFillColor(dropdownText);
    sf::FloatRect textBounds = dropdown.mainLabel.getLocalBounds();
    dropdown.mainLabel.setPosition(rightPanelX + 10, startY + (dropdownHeight - textBounds.height) / 2.0f - textBounds.top);

    for (int i = 0; i < options.size(); i++)
    {
        sf::RectangleShape option;
        option.setSize(sf::Vector2f(dropdownWidth, dropdownHeight));
        option.setFillColor(dropdownBg);
        option.setOutlineThickness(1);
        option.setOutlineColor(sf::Color(180, 180, 180));
        option.setPosition(rightPanelX, startY + (i + 1) * dropdownHeight);

        sf::Text optionLabel;
        optionLabel.setFont(font);
        optionLabel.setString(options[i]);
        optionLabel.setCharacterSize(14);
        optionLabel.setFillColor(dropdownText);
        textBounds = optionLabel.getLocalBounds();
        optionLabel.setPosition(
            rightPanelX + 10,
            startY + (i + 1) * dropdownHeight + (dropdownHeight - textBounds.height) / 2.0f - textBounds.top);

        dropdown.options.push_back(option);
        dropdown.optionLabels.push_back(optionLabel);
        dropdown.optionNames.push_back(options[i]);
    }

    return dropdown;
}

unsigned char board[8][8] = {{'-'}};

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
}

std::vector<sf::Sprite> makePieces(const sf::Texture &texture, std::string fen)
{
    std::vector<sf::Sprite> result;
    int row = 0, col = 0;
    sf::Sprite piece(texture);
    piece.setScale(SQUARE_SIZE / 64.f, SQUARE_SIZE / 64.f);

    for (char c : fen)
    {
        if (isdigit(c))
        {
            col += c - '0';
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
    return result;
}

void drawTitle(sf::RenderWindow &window, sf::Font &font)
{
    sf::Text title;
    title.setFont(font);
    title.setString("Pawn Stars");
    title.setCharacterSize(22);
    title.setFillColor(sf::Color(50, 50, 50));
    title.setStyle(sf::Text::Bold);
    sf::FloatRect textBounds = title.getLocalBounds();
    title.setPosition((window.getSize().x - textBounds.width) / 2.0f - textBounds.left, 12);
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

class ChessSubscriber : public rclcpp::Node
{
public:
    ChessSubscriber() : Node("chess_visualizer")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "chess_moves", 10, std::bind(&ChessSubscriber::moveCallback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::String>("chess_control", 10);

        reset_client_ = this->create_client<std_srvs::srv::Trigger>("/reset_chessboard");

        button_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "button_state", 10, [this](std_msgs::msg::Bool::SharedPtr msg)
            {
                buttonState = msg->data;
                RCLCPP_INFO(this->get_logger(), "Button state: %s", buttonState ? "ON" : "OFF"); });
    }

    void callResetService()
    {
        if (!reset_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Reset service not available.");
            return;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = reset_client_->async_send_request(request);

        try
        {
            auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "Reset successful: %s", response->message.c_str());
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Reset failed: %s", response->message.c_str());
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
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

private:
    void moveCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        lastMove = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received move: %s", lastMove.c_str());
    }

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_client_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr button_state_sub_;
    std::string lastMove;
    bool buttonState = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChessSubscriber>();
    std::string fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR";
    std::string status = "Ready to play";

    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Chess Visualizer", sf::Style::Titlebar | sf::Style::Close);
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

    std::vector<Button> buttons = createButtons(font);
    Dropdown difficultyDropdown = createDifficultyDropdown(font);

    while (window.isOpen() && rclcpp::ok())
    {
        rclcpp::spin_some(node);
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
            if (event.type == sf::Event::MouseMoved)
            {
                sf::Vector2f mousePos(event.mouseMove.x, event.mouseMove.y);
                for (auto &button : buttons)
                {
                    bool wasHovered = button.isHovered;
                    button.isHovered = button.shape.getGlobalBounds().contains(mousePos);
                    if (button.isHovered != wasHovered)
                    {
                        button.shape.setFillColor(button.isHovered ? buttonHover : buttonBlue);
                    }
                }
            }
            if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left)
            {
                sf::Vector2f mousePos(event.mouseButton.x, event.mouseButton.y);
                for (const auto &button : buttons)
                {
                    if (button.shape.getGlobalBounds().contains(mousePos))
                    {
                        if (button.name == "RESET")
                        {
                            status = "Resetting chessboard...";
                            node->callResetService();
                        }
                        else
                        {
                            status = "Command: " + button.name;
                            node->publishControl(button.name);
                        }
                    }
                }
                if (difficultyDropdown.mainButton.getGlobalBounds().contains(mousePos))
                {
                    difficultyDropdown.isOpen = !difficultyDropdown.isOpen;
                }
                else if (difficultyDropdown.isOpen)
                {
                    for (int i = 0; i < difficultyDropdown.options.size(); i++)
                    {
                        if (difficultyDropdown.options[i].getGlobalBounds().contains(mousePos))
                        {
                            difficultyDropdown.selectedIndex = i;
                            difficultyDropdown.mainLabel.setString("Difficulty: " + difficultyDropdown.optionNames[i]);
                            difficultyDropdown.isOpen = false;
                            sf::FloatRect textBounds = difficultyDropdown.mainLabel.getLocalBounds();
                            float x = difficultyDropdown.mainButton.getPosition().x + 10;
                            float y = difficultyDropdown.mainButton.getPosition().y +
                                      (difficultyDropdown.mainButton.getSize().y - textBounds.height) / 2.0f - textBounds.top;
                            difficultyDropdown.mainLabel.setPosition(x, y);
                            node->publishControl("DIFFICULTY_" + difficultyDropdown.optionNames[i]);
                            status = "Difficulty set to " + difficultyDropdown.optionNames[i];
                        }
                    }
                }
            }
        }

        window.clear(sf::Color(250, 250, 250));
        drawTitle(window, font);
        drawBoard(window);
        for (const auto &piece : pieces)
            window.draw(piece);
        for (const auto &button : buttons)
        {
            window.draw(button.shape);
            window.draw(button.label);
        }
        window.draw(difficultyDropdown.mainButton);
        window.draw(difficultyDropdown.mainLabel);
        if (difficultyDropdown.isOpen)
        {
            for (int i = 0; i < difficultyDropdown.options.size(); i++)
            {
                window.draw(difficultyDropdown.options[i]);
                window.draw(difficultyDropdown.optionLabels[i]);
            }
        }

        drawStatus(window, font, status);

        // Indicator light
        sf::RectangleShape indicator(sf::Vector2f(40, 40));
        indicator.setPosition(WINDOW_WIDTH - 50, 10);
        indicator.setFillColor(node->getButtonState() ? buttonBlue : sf::Color::Red);
        indicator.setOutlineColor(sf::Color::Black);
        indicator.setOutlineThickness(1);
        window.draw(indicator);

        sf::Text indicatorLabel;
        indicatorLabel.setFont(font);
        indicatorLabel.setCharacterSize(16);
        indicatorLabel.setFillColor(sf::Color(50, 50, 50));

        if (node->getButtonState())
        {
            // Robot is UNLOCKED
            indicatorLabel.setString("Robot unlocked: ");
            indicatorLabel.setPosition(WINDOW_WIDTH - 220, 22);
        }
        else
        {
            indicatorLabel.setString("Robot locked: ");
            indicatorLabel.setPosition(WINDOW_WIDTH - 200, 22);
        }

        window.draw(indicatorLabel);
        window.display();
    }

    rclcpp::shutdown();
    return 0;
}