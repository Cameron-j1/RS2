#include <iostream>
#include <SFML/Graphics.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <vector>
#include <string>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>

// Configs
const int BOARD_SIZE = 8;
const int SQUARE_SIZE = 45;
const int BOARD_OFFSET_X = 25;
const int BOARD_OFFSET_Y = 20;
const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 460;

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

// Flashing text struct
struct FlashingText {
    sf::Text text;
    bool isActive = false;
    bool isVisible = true;
    float frequency = 2.0f; // Flashes per second
    float timer = 0.0f;
};

#pragma region GUI Structs and Classes

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
    std::vector<std::string> labels = {""};

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

// Create a flashing text message
FlashingText createFlashingText(sf::Font &font, const std::string &message, float x, float y, float frequency, sf::Color color)
{
    FlashingText flashingText;
    
    flashingText.text.setFont(font);
    flashingText.text.setString(message);
    flashingText.text.setCharacterSize(20);
    flashingText.text.setFillColor(color);
    flashingText.text.setStyle(sf::Text::Bold);
    
    // Center the text
    sf::FloatRect textBounds = flashingText.text.getLocalBounds();
    flashingText.text.setPosition(
        x - textBounds.width / 2.0f - textBounds.left,
        y - textBounds.height / 2.0f - textBounds.top);
    
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
    title.setPosition((window.getSize().x - textBounds.width) / 2.0f - textBounds.left +80, 8);
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

#pragma endregion GUI

#pragma region ROS2 Topic Subs and Services
class ChessSubscriber : public rclcpp::Node
{
public:
    ChessSubscriber() : Node("chess_visualizer")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "fen_string", 10, std::bind(&ChessSubscriber::moveCallback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::String>("chess_control", 10);

        reset_client_ = this->create_client<std_srvs::srv::Trigger>("/reset_chessboard");

        button_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "button_state", 10, [this](std_msgs::msg::Bool::SharedPtr msg)
            {
                buttonState = msg->data;
                RCLCPP_INFO(this->get_logger(), "Button state: %s", buttonState ? "ON" : "OFF"); });
                
        // Add new subscribers for board_oor and player_turn_bool
        board_oor_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "board_oor", 10, [this](std_msgs::msg::Bool::SharedPtr msg)
            {
                boardOutOfRange = msg->data;
                RCLCPP_INFO(this->get_logger(), "Board out of range: %s", boardOutOfRange ? "YES" : "NO");
            });
            
        player_turn_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "player_turn_bool", 10, [this](std_msgs::msg::Bool::SharedPtr msg)
            {
                playerTurn = msg->data;
                RCLCPP_INFO(this->get_logger(), "Player turn: %s", playerTurn ? "YES" : "NO");
            });
        estop_flag_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "estop_flag", 10, [this](std_msgs::msg::Bool::SharedPtr msg)
            {
                estopFlag = msg->data;
                RCLCPP_INFO(this->get_logger(), "Player turn: %s", estopFlag ? "YES" : "NO");
            });

        status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/status", 10, [this](std_msgs::msg::String::SharedPtr msg)
            {
                if (msg->data.size() >= 4) {
                    temp1 = msg->data[0];
                    temp2 = msg->data[1];
                    temp3 = msg->data[2];
                    temp4 = msg->data[3];
                    RCLCPP_INFO(this->get_logger(), "Status received: %c %c %c %c", temp1, temp2, temp3, temp4);
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
    bool getBoardOutOfRange() const { return boardOutOfRange; }
    bool getPlayerTurn() const { return playerTurn; }
    bool getEstopFlag() const { return estopFlag; }

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
    bool boardOutOfRange = false;
    bool playerTurn = false;
    bool estopFlag = false;
};
#pragma endregion ROS



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
        WINDOW_HEIGHT/ 2 + 0, 
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
                        node->publishControl("DIFFICULTY_" + selectedDifficulty);
                        status = "Game started with " + selectedDifficulty + " difficulty";
                        // Reset the board via ROS service
                        node->callResetService();
                    }
                } else if (currentState == GAME_SCREEN) {
                    // Check for game button clicks
                    for (const auto &button : gameButtons) {
                        if (button.shape.getGlobalBounds().contains(mousePos)) {
                            status = "Command: " + button.name;
                            node->publishControl(button.name);
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
            // Update board out of range warning
            boardOutOfRangeText.isActive = node->getBoardOutOfRange();
            updateFlashingText(boardOutOfRangeText, deltaTime);
            
            // Update player turn notification
            playerTurnText.isActive = node->getPlayerTurn();
            updateFlashingText(playerTurnText, deltaTime);

            eStopEngaged.isActive = node->getEstopFlag();
            updateFlashingText(eStopEngaged, deltaTime);

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

        window.clear(sf::Color(250, 250, 250));
        drawTitle(window, font);
        
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
            
        } else if (currentState == GAME_SCREEN) {
            // Draw game screen elements
            drawBoard(window);
            for (const auto &piece : pieces)
                window.draw(piece);
                
            for (const auto &button : gameButtons) {
                window.draw(button.shape);
                window.draw(button.label);
            }
            
            drawStatus(window, font, status);
            
            // Draw flashing texts if they're active and visible
            if (boardOutOfRangeText.isActive && boardOutOfRangeText.isVisible) {
                window.draw(boardOutOfRangeText.text);
            }
            
            if (playerTurnText.isActive && playerTurnText.isVisible) {
                window.draw(playerTurnText.text);
            }
            
            if (eStopEngaged.isActive && eStopEngaged.isVisible) {
                window.draw(eStopEngaged.text);
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
                indicatorLabel.setString("Robot unlocked: ");
                indicatorLabel.setPosition(WINDOW_WIDTH - 220, 22);
            } else {
                indicatorLabel.setString("Robot locked: ");
                indicatorLabel.setPosition(WINDOW_WIDTH - 200, 22);
            }

            window.draw(indicatorLabel);
            
            // Display current difficulty in game screen
            sf::Text difficultyText;
            difficultyText.setFont(font);
            difficultyText.setString("Difficulty: " + selectedDifficulty);
            difficultyText.setCharacterSize(14);
            difficultyText.setFillColor(sf::Color(80, 80, 80));
            difficultyText.setPosition(BOARD_OFFSET_X + BOARD_SIZE * SQUARE_SIZE + 260, BOARD_OFFSET_Y + 280);
            window.draw(difficultyText);
        }
        
        window.display();
    }

    rclcpp::shutdown();
    return 0;
}


#pragma endregion Main Function

//TODO 
// Assign GUI elemenst to the Error flags currently temp1, temp2, temp3, temp4

//