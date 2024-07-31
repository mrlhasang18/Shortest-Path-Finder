



// ==============================================================================================================================================================================================================
//
//                                                                                          PATH FINDING VISUALIZER
//
// ==============================================================================================================================================================================================================
//
// This visualizer helps in understanding pathfinding algorithms by visualizing their
// execution on a grid and a maze.
//
// ==============================================================================================================================================================================================================








// ---------------------------------------------------------------------------------------------- Header files -------------------------------------------------------------------------------------------

#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <thread>
#include "algorithms.h"
#include "maze.h"

using namespace std;
using namespace sf;
const int num = 60; // Number of cells in a row

// ----------------------------------------------------------------------------------------------- Global variables ----------------------------------------------------------------------------------------

vector<pair<int, int>> path; // Shortest path
vector<pair<int, int>> exploredCells; // Cells explored during pathfinding
bool explored[num][num]; // Explored cells
int grid[num][num]; // Map with obstacle
bool isPathFound = false;
string algorithmUsed = "";
int stepsTaken = 0;
float timeTaken = 0.0f;
int source_x = 2, source_y = 2, dest_x = 50, dest_y = 56; // Origin and Goal coordinates
bool isSettingStart = false;
bool isSettingDest = false;

// variables for gradual updating
chrono::high_resolution_clock::time_point startTime;
bool isSearching = false;
string statusMessage = "";

// variable to track if the destination has been reached
bool destinationReached = false;
chrono::high_resolution_clock::time_point visualizationStartTime;
bool isVisualizationTimeRunning = false;

// Function to clear the path
void clearPath() {
    path.clear();
    exploredCells.clear();
    memset(explored, false, sizeof(explored));
    isPathFound = false;
    algorithmUsed = "";
    stepsTaken = 0;
    timeTaken = 0.0f;
    isSearching = false;
    statusMessage = "";
}



// ------------------------------------------------------------------------------------------------- Main function --------------------------------------------------------------------------------


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
    
    
// --------------------------------------------------------------------------------- Load Fonts --------------------------------------------------------------------
    Font font;
    if (!font.loadFromFile("arial.ttf")) {
        cerr << "Failed to load font" << endl;
        return -1;
    }



// --------------------------------------------------------------------------------- Text and Buttons --------------------------------------------------------------------


    // Create text objects
    Text textTitle("Pathfinding Visualizer", font, 24);
    Text textDijkstra("Dijkstra", font, 15);
    Text textAStar("A*", font, 15);
    Text textCreateMaze("Create Maze", font, 15);
    Text textClearMaze("Clear Board", font, 15);
    Text textSetStart("Set Start", font, 15);
    Text textSetDest("Set Destination", font, 15);
    Text textInfo("", font, 12);

    // Create buttons
    RectangleShape buttonDijkstra(Vector2f(75, 25));
    RectangleShape buttonAStar(Vector2f(75, 25));
    RectangleShape buttonCreateMaze(Vector2f(100, 25));
    RectangleShape buttonClearMaze(Vector2f(100, 25));
    RectangleShape buttonSetStart(Vector2f(75, 25));
    RectangleShape buttonSetDest(Vector2f(120, 25));
    RectangleShape infoContainer(Vector2f(200, 80));

    // Set button colors
    buttonDijkstra.setFillColor(Color::Green);
    buttonAStar.setFillColor(Color::Magenta);
    buttonCreateMaze.setFillColor(Color::Black);
    buttonClearMaze.setFillColor(Color::Red);
    buttonSetStart.setFillColor(Color::Blue);
    buttonSetDest.setFillColor(Color::Red);
    infoContainer.setFillColor(Color::White);
    infoContainer.setOutlineColor(Color::Black);
    infoContainer.setOutlineThickness(2);

    // Set text and button positions
    textTitle.setPosition(610, 10);
    textTitle.setFillColor(Color::Black);

    buttonDijkstra.setPosition(610, 50);
    buttonAStar.setPosition(610, 85);
    buttonCreateMaze.setPosition(610, 120);
    infoContainer.setPosition(610, 155);
    buttonClearMaze.setPosition(610, 245);
    buttonSetStart.setPosition(610, 280);
    buttonSetDest.setPosition(610, 315);

    textDijkstra.setPosition(620, 55);
    textAStar.setPosition(635, 90);
    textCreateMaze.setPosition(615, 125);
    textClearMaze.setPosition(620, 250);
    textSetStart.setPosition(620, 285);
    textSetDest.setPosition(615, 320);


// --------------------------------------------------------------------------------- Grid and Cells  --------------------------------------------------------------------
    // Create cell shapes
    RectangleShape cellShape(Vector2f(10, 10));
    cellShape.setOutlineThickness(1);
    cellShape.setOutlineColor(Color::Black);

    bool drawing = false;
    bool isVisualizing = false;
    size_t visualizationIndex = 0;
    Vector2i lastPos;
    
    while (window.isOpen()) {
        Event event;
        while (window.pollEvent(event)) {
            if (event.type == Event::Closed) {
                window.close();
                return 0;  // Exit the program
            }
            if (event.type == Event::KeyPressed && event.key.code == Keyboard::Escape) {
                window.close();
                return 0;  // Exit the program
            }
            if (event.type == Event::MouseButtonPressed && event.mouseButton.button == Mouse::Left) {
                int X = event.mouseButton.x;
                int Y = event.mouseButton.y;
                
                if (X < 600 && Y < 600) {
                    int row = Y / 10;
                    int col = X / 10;
                    if (row < num && col < num) {
                        if (isSettingStart) {
                            source_x = row;
                            source_y = col;
                            isSettingStart = false;
                        } else if (isSettingDest) {
                            dest_x = row;
                            dest_y = col;
                            isSettingDest = false;
                        } else {
                            grid[row][col] = (grid[row][col] == 0) ? 1 : 0;
                            drawing = true;
                            lastPos = Vector2i(col, row);
                        }
                    }
                } else if (buttonDijkstra.getGlobalBounds().contains(X, Y)) {
                    // Check if destination is reachable
                    if (grid[dest_x][dest_y] == 0) {
                        statusMessage = "Destination is unreachable!";
                    } else {
                        dijkstra(source_x, source_y, dest_x, dest_y);
                        isVisualizing = true;
                        visualizationIndex = 0;
                        visualizationStartTime = chrono::high_resolution_clock::now();
                        isVisualizationTimeRunning = true;
                        stepsTaken = 0;
                        timeTaken = 0.0f;
                    }
                } else if (buttonAStar.getGlobalBounds().contains(X, Y)) {
                    // Check if destination is reachable
                    if (grid[dest_x][dest_y] == 0) {
                        statusMessage = "Destination is unreachable!";
                    } else {
                        astar(source_x, source_y, dest_x, dest_y);
                        isVisualizing = true;
                        visualizationIndex = 0;
                        visualizationStartTime = chrono::high_resolution_clock::now();
                        isVisualizationTimeRunning = true;
                        stepsTaken = 0;
                        timeTaken = 0.0f;
                    }
                } else if (buttonCreateMaze.getGlobalBounds().contains(X, Y)) {
                    createMaze();
                } else if (buttonClearMaze.getGlobalBounds().contains(X, Y)) {
                    clearBoard();
                } else if (buttonSetStart.getGlobalBounds().contains(X, Y)) {
                    isSettingStart = true;
                    isSettingDest = false;
                } else if (buttonSetDest.getGlobalBounds().contains(X, Y)) {
                    isSettingDest = true;
                    isSettingStart = false;
                }
            }



 // --------------------------------------------------------------------------------- User Interactions --------------------------------------------------------------------
 
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
                        drawLine(lastPos.x, lastPos.y, col, row, grid[lastPos.y][lastPos.x] == 0 ? 0 : 1);
                        lastPos = Vector2i(col, row);
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
                } else {
                    cellShape.setFillColor(Color::White);
                }
                window.draw(cellShape);
            }
        }

 // --------------------------------------------------------------------------------- Visualizing Exploration and final path --------------------------------------------------------------------
        if (isVisualizing) {
        	
            // Speed up visualization by processing multiple cells per frame
            for (int i = 0; i < 10; i++) {  // Process 10 cells per frame
                if (visualizationIndex < exploredCells.size()) {
                    int x = exploredCells[visualizationIndex].first;
                    int y = exploredCells[visualizationIndex].second;
                    cellShape.setPosition(y * 10, x * 10);
                    cellShape.setFillColor(Color::Yellow);  // Set explored cells to yellow
                    window.draw(cellShape);
                    visualizationIndex++;
                    
                    // Update steps in real-time
                    stepsTaken = visualizationIndex;

                    // Check if destination is reached
                    if (x == dest_x && y == dest_y) {
                        destinationReached = true;
                        isVisualizationTimeRunning = false;
                    }
                } else if (visualizationIndex < exploredCells.size() + path.size()) {
                    size_t pathIndex = visualizationIndex - exploredCells.size();
                    int x = path[pathIndex].first;
                    int y = path[pathIndex].second;
                    cellShape.setPosition(y * 10, x * 10);
                    cellShape.setFillColor(algorithmUsed == "Dijkstra" ? Color::Green : Color::Magenta);
                    window.draw(cellShape);
                    visualizationIndex++;
                } else {
                    isVisualizing = false;
                    isSearching = false;
                    isVisualizationTimeRunning = false;
                    break;
                }
            }
        } else {
            // --------------------------------Draw explored cells --------------------
            for (const auto& cell : exploredCells) {
                int x = cell.first;
                int y = cell.second;
                cellShape.setPosition(y * 10, x * 10);
                cellShape.setFillColor(Color::Yellow);  // Set explored cells to yellow
                window.draw(cellShape);
            }

            // -------------------------------- Draw path -----------------------------
            for (const auto& cell : path) {
                int x = cell.first;
                int y = cell.second;
                cellShape.setPosition(y * 10, x * 10);
                cellShape.setFillColor(algorithmUsed == "Dijkstra" ? Color::Green : Color::Magenta);
                window.draw(cellShape);
            }
        }

// --------------------------------------------------------------------------------- Draw Source and Destination -------------------------------------------------------------------------------

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
        window.draw(buttonClearMaze);
        window.draw(buttonSetStart);
        window.draw(buttonSetDest);
        window.draw(infoContainer);

        // Draw texts
        window.draw(textTitle);
        window.draw(textDijkstra);
        window.draw(textAStar);
        window.draw(textCreateMaze);
        window.draw(textClearMaze);
        window.draw(textSetStart);
        window.draw(textSetDest);

        // Update and draw info text
        stringstream ss;
        if (isSearching || isPathFound) {
            ss << "Algorithm: " << algorithmUsed << "\n";
            ss << "Steps: " << stepsTaken << "\n";
            
// ----------------------------------------------------------------------------- Update time in real-time, but stop when destination is reached -------------------------------------------------
            
            if (isVisualizationTimeRunning) {
                auto currentTime = chrono::high_resolution_clock::now();
                chrono::duration<float> elapsedTime = currentTime - visualizationStartTime;
                timeTaken = elapsedTime.count();
            }
            
            ss << "Time: " << fixed << setprecision(2) << timeTaken << " seconds\n";
            ss << statusMessage;
        } else {
            ss << statusMessage;
        }
        textInfo.setString(ss.str());
        textInfo.setPosition(615, 160);
        textInfo.setFillColor(Color::Black);
        window.draw(textInfo);

        window.display();

        // ------------------------------------ Control visualization speed -----------------------
        if (isVisualizing) {
            this_thread::sleep_for(chrono::milliseconds(10));  // Reduced sleep time for faster visualization
        }
    }

    return 0;
}
