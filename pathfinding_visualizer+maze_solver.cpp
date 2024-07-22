#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <iostream>
#include <functional>
#include <cmath>
#include <cfloat>
#include <vector>
#include <set>
#include <cstring>
#include <string>
#include <sstream>
#include <chrono>
#include <random>
#include <queue>
#include <iomanip>

using namespace std;
using namespace sf;

#define num 60 // Number of cells in a row

// Global variables
vector<pair<int, int>> path; // Shortest path
bool explored[num][num]; // Explored cells
int grid[num][num]; // Map with obstacle
bool isPathFound = false;
string algorithmUsed = "";
int stepsTaken = 0;
float timeTaken = 0.0f;
int source_x = 2, source_y = 2, dest_x = 50, dest_y = 56; // Origin and Goal coordinates

// Function to clear the path
void clearPath() {
    path.clear();
    memset(explored, false, sizeof(explored));
    isPathFound = false;
    algorithmUsed = "";
    stepsTaken = 0;
    timeTaken = 0.0f;
}

// Function to clear obstacles
void clearObstacles() {
    for (int i = 1; i < num - 1; i++) {
        for (int j = 1; j < num - 1; j++) {
            grid[i][j] = 1;
        }
    }
    clearPath();
}

// Function to clear the entire board
void clearBoard() {
    clearObstacles();
    clearPath();
}

// Function to create a maze using Recursive Backtracking algorithm
void createMaze() {
    clearObstacles();
    
    // Set all cells as walls initially
    for (int i = 0; i < num; i++) {
        for (int j = 0; j < num; j++) {
            grid[i][j] = 0;
        }
    }

    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> dis(0, 3);

    vector<pair<int, int>> stack;
    int startX = 1, startY = 1;
    grid[startX][startY] = 1;
    stack.push_back({startX, startY});

    while (!stack.empty()) {
        int x = stack.back().first;
        int y = stack.back().second;
        vector<pair<int, int>> neighbors;

        // Check neighbors (only vertical and horizontal)
        int dx[] = {0, 0, 1, -1};
        int dy[] = {1, -1, 0, 0};

        for (int i = 0; i < 4; i++) {
            int nx = x + dx[i] * 2;
            int ny = y + dy[i] * 2;
            if (nx > 0 && nx < num - 1 && ny > 0 && ny < num - 1 && grid[nx][ny] == 0) {
                neighbors.push_back({nx, ny});
            }
        }

        if (!neighbors.empty()) {
            int index = dis(gen) % neighbors.size();
            int nx = neighbors[index].first;
            int ny = neighbors[index].second;
            grid[nx][ny] = 1;
            grid[x + (nx - x) / 2][y + (ny - y) / 2] = 1;
            stack.push_back({nx, ny});
        } else {
            stack.pop_back();
        }
    }

    // Ensure start and end points are in open spaces
    uniform_int_distribution<> dis_coord(1, num - 2);
    do {
        source_x = dis_coord(gen);
        source_y = dis_coord(gen);
    } while (grid[source_x][source_y] == 0);

    do {
        dest_x = dis_coord(gen);
        dest_y = dis_coord(gen);
    } while (grid[dest_x][dest_y] == 0 || (dest_x == source_x && dest_y == source_y));

    clearPath();
}

// Dijkstra's algorithm
void dijkstra(int source_x, int source_y, int dest_x, int dest_y) {
    clearPath();
    auto start = chrono::high_resolution_clock::now();
    
    priority_queue<pair<float, pair<int, int>>, vector<pair<float, pair<int, int>>>, greater<pair<float, pair<int, int>>>> pq;
    vector<vector<float>> dist(num, vector<float>(num, FLT_MAX));
    vector<vector<pair<int, int>>> previous(num, vector<pair<int, int>>(num, {-1, -1}));

    dist[source_x][source_y] = 0.0;
    pq.push({0.0, {source_x, source_y}});

    while (!pq.empty()) {
        int x = pq.top().second.first;
        int y = pq.top().second.second;
        pq.pop();

        explored[x][y] = true;
        stepsTaken++;

        if (x == dest_x && y == dest_y) {
            break;
        }

        int dx[] = {0, 0, 1, -1};
        int dy[] = {1, -1, 0, 0};

        for (int i = 0; i < 4; i++) {
            int nx = x + dx[i];
            int ny = y + dy[i];
            if (nx >= 0 && nx < num && ny >= 0 && ny < num && grid[nx][ny] == 1 && !explored[nx][ny]) {
                float newDist = dist[x][y] + 1.0;
                if (newDist < dist[nx][ny]) {
                    dist[nx][ny] = newDist;
                    previous[nx][ny] = {x, y};
                    pq.push({newDist, {nx, ny}});
                }
            }
        }
    }

    // Reconstruct path
    int x = dest_x, y = dest_y;
    while (x != source_x || y != source_y) {
        path.push_back({x, y});
        int px = previous[x][y].first;
        int py = previous[x][y].second;
        x = px;
        y = py;
    }
    path.push_back({source_x, source_y});
    reverse(path.begin(), path.end());

    auto end = chrono::high_resolution_clock::now();
    chrono::duration<float> duration = end - start;
    timeTaken = duration.count();
    
    isPathFound = true;
    algorithmUsed = "Dijkstra";
}

// A* algorithm
void astar(int source_x, int source_y, int dest_x, int dest_y) {
    clearPath();
    auto start = chrono::high_resolution_clock::now();
    
    auto heuristic = [](int x1, int y1, int x2, int y2) {
        return abs(x1 - x2) + abs(y1 - y2);
    };

    priority_queue<pair<float, pair<int, int>>, vector<pair<float, pair<int, int>>>, greater<pair<float, pair<int, int>>>> pq;
    vector<vector<float>> g_score(num, vector<float>(num, FLT_MAX));
    vector<vector<float>> f_score(num, vector<float>(num, FLT_MAX));
    vector<vector<pair<int, int>>> previous(num, vector<pair<int, int>>(num, {-1, -1}));

    g_score[source_x][source_y] = 0.0;
    f_score[source_x][source_y] = heuristic(source_x, source_y, dest_x, dest_y);
    pq.push({f_score[source_x][source_y], {source_x, source_y}});

    while (!pq.empty()) {
        int x = pq.top().second.first;
        int y = pq.top().second.second;
        pq.pop();

        explored[x][y] = true;
        stepsTaken++;

        if (x == dest_x && y == dest_y) {
            break;
        }

        int dx[] = {0, 0, 1, -1};
        int dy[] = {1, -1, 0, 0};

        for (int i = 0; i < 4; i++) {
            int nx = x + dx[i];
            int ny = y + dy[i];
            if (nx >= 0 && nx < num && ny >= 0 && ny < num && grid[nx][ny] == 1 && !explored[nx][ny]) {
                float tentative_g_score = g_score[x][y] + 1.0;
                if (tentative_g_score < g_score[nx][ny]) {
                    previous[nx][ny] = {x, y};
                    g_score[nx][ny] = tentative_g_score;
                    f_score[nx][ny] = g_score[nx][ny] + heuristic(nx, ny, dest_x, dest_y);
                    pq.push({f_score[nx][ny], {nx, ny}});
                }
            }
        }
    }

    // Reconstruct path
    int x = dest_x, y = dest_y;
    while (x != source_x || y != source_y) {
        path.push_back({x, y});
        int px = previous[x][y].first;
        int py = previous[x][y].second;
        x = px;
        y = py;
    }
    path.push_back({source_x, source_y});
    reverse(path.begin(), path.end());

    auto end = chrono::high_resolution_clock::now();
    chrono::duration<float> duration = end - start;
    timeTaken = duration.count();
    
    isPathFound = true;
    algorithmUsed = "A*";
}

int main() {
    // Initialize grid
    for (int i = 0; i < num; i++) {
        for (int j = 0; j < num; j++) {
            if (i == 0 || i == num - 1 || j == 0 || j == num - 1) {
                grid[i][j] = 0; // Walls
            } else {
                grid[i][j] = 1; // Open cells
            }
        }
    }

    RenderWindow window(VideoMode(1000, 600), "Pathfinding Visualizer");

    // Load font
    Font font;
    if (!font.loadFromFile("arial.ttf")) {
        cerr << "Failed to load font" << endl;
        return -1;
    }

    // Create text objects
    Text textDijkstra("Dijkstra", font, 15);
    Text textAStar("A*", font, 15);
    Text textCreateMaze("Create Maze", font, 15);
    Text textClearObstacles("Clear Obstacles", font, 15);
    Text textClearMaze("Clear Maze", font, 15);
    Text textClearBoard("Clear Board", font, 15);
    Text textInfo("", font, 12);

    // Create buttons
    RectangleShape buttonDijkstra(Vector2f(75, 25));
    RectangleShape buttonAStar(Vector2f(75, 25));
    RectangleShape buttonCreateMaze(Vector2f(100, 25));
    RectangleShape buttonClearObstacles(Vector2f(120, 25));
    RectangleShape buttonClearMaze(Vector2f(100, 25));
    RectangleShape buttonClearBoard(Vector2f(100, 25));
    RectangleShape infoContainer(Vector2f(200, 80));

    // Set button colors
    buttonDijkstra.setFillColor(Color::Green);
    buttonAStar.setFillColor(Color::Magenta);
    buttonCreateMaze.setFillColor(Color::Cyan);
    buttonClearObstacles.setFillColor(Color::Yellow);
    buttonClearMaze.setFillColor(Color::Red);
    buttonClearBoard.setFillColor(Color::White);
    infoContainer.setFillColor(Color::White);
    infoContainer.setOutlineColor(Color::Black);
    infoContainer.setOutlineThickness(2);

    // Set button positions
    buttonDijkstra.setPosition(610, 10);
    buttonAStar.setPosition(610, 45);
    buttonCreateMaze.setPosition(610, 80);
    infoContainer.setPosition(610, 115);
    buttonClearObstacles.setPosition(610, 205);
    buttonClearMaze.setPosition(610, 240);
    buttonClearBoard.setPosition(610, 275);

    // Set text positions
    textDijkstra.setPosition(620, 15);
    textAStar.setPosition(635, 50);
    textCreateMaze.setPosition(615, 85);
    textClearObstacles.setPosition(615, 210);
    textClearMaze.setPosition(620, 245);
    textClearBoard.setPosition(620, 280);

    // Create cell shapes
    RectangleShape cellShape(Vector2f(10, 10));
    cellShape.setOutlineThickness(1);
    cellShape.setOutlineColor(Color::Black);

    bool drawing = false;

    while (window.isOpen()) {
        Event event;
        while (window.pollEvent(event)) {
            if (event.type == Event::Closed)
                window.close();
            if (event.type == Event::KeyPressed && event.key.code == Keyboard::Escape)
                window.close();
            
            if (event.type == Event::MouseButtonPressed && event.mouseButton.button == Mouse::Left) {
                int X = event.mouseButton.x;
                int Y = event.mouseButton.y;
                
                if (X < 600 && Y < 600) {
                    int row = Y / 10;
                    int col = X / 10;
                    if (row < num && col < num) {
                        grid[row][col] = (grid[row][col] == 0) ? 1 : 0;
                        drawing = true;
                    }
                } else if (buttonDijkstra.getGlobalBounds().contains(X, Y)) {
                    dijkstra(source_x, source_y, dest_x, dest_y);
                } else if (buttonAStar.getGlobalBounds().contains(X, Y)) {
                    astar(source_x, source_y, dest_x, dest_y);
                } else if (buttonCreateMaze.getGlobalBounds().contains(X, Y)) {
                    createMaze();
                } else if (buttonClearObstacles.getGlobalBounds().contains(X, Y)) {
                    clearObstacles();
                } else if (buttonClearMaze.getGlobalBounds().contains(X, Y)) {
                    clearObstacles();
                } else if (buttonClearBoard.getGlobalBounds().contains(X, Y)) {
                    clearBoard();
                }
            }
            
            if (event.type == Event::MouseButtonReleased && event.mouseButton.button == Mouse::Left) {
                drawing = false;
            }
            
            if (event.type == Event::MouseMoved && drawing) {
                int X = event.mouseMove.x;
                int Y = event.mouseMove.y;
                if (X < 600 && Y < 600) {
                    int row = Y / 10;
                    int col = X / 10;
                    if (row < num && col < num) {
                        grid[row][col] = (grid[row][col] == 0) ? 1 : 0;
                    }
                }
            }
        }

        window.clear(Color::White);

        // Draw grid
        for (int i = 0; i < num; i++) {
            for (int j = 0; j < num; j++) {
                cellShape.setPosition(j * 10, i * 10);
                if (grid[i][j] == 0) {
                    cellShape.setFillColor(Color::Black);
                } else if (explored[i][j]) {
                    cellShape.setFillColor(Color::Yellow);
                } else {
                    cellShape.setFillColor(Color::White);
                }
                window.draw(cellShape);
            }
        }

        // Draw path
        for (const auto& p : path) {
            cellShape.setPosition(p.second * 10, p.first * 10);
            cellShape.setFillColor(Color::Green);
            window.draw(cellShape);
        }

        // Draw source and destination
        cellShape.setPosition(source_y * 10, source_x * 10);
        cellShape.setFillColor(Color::Blue);
        window.draw(cellShape);
        cellShape.setPosition(dest_y * 10, dest_x * 10);
        cellShape.setFillColor(Color::Red);
        window.draw(cellShape);

        // Draw buttons
        window.draw(buttonDijkstra);
        window.draw(buttonAStar);
        window.draw(buttonCreateMaze);
        window.draw(buttonClearObstacles);
        window.draw(buttonClearMaze);
        window.draw(buttonClearBoard);
        window.draw(infoContainer);

        // Draw button texts
        window.draw(textDijkstra);
        window.draw(textAStar);
        window.draw(textCreateMaze);
        window.draw(textClearObstacles);
        window.draw(textClearMaze);
        window.draw(textClearBoard);

        // Update and draw info text
        if (isPathFound) {
            stringstream ss;
            ss << "Algorithm: " << algorithmUsed << "\n";
            ss << "Steps: " << stepsTaken << "\n";
            ss << "Time: " << fixed << setprecision(6) << timeTaken << " seconds";
            textInfo.setString(ss.str());
        } else {
            textInfo.setString("No path found yet");
        }
        textInfo.setPosition(615, 120);
        textInfo.setFillColor(Color::Black);
        window.draw(textInfo);

        window.display();
    }

    return 0;
}
