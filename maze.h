

// ==============================================================================================================================================================================================================
//
//                                                                                          MAZE.H
//
// ==============================================================================================================================================================================================================
//

#ifndef MAZE_H
#define MAZE_H

// -------------------------------------------------------------------------------------  Header files ---------------------------------------------------------------
#include <vector>
#include <random>
#include <cstring>

extern const int num;
extern int grid[60][60];

void clearBoard() {
    for (int i = 1; i < num - 1; i++) {
        for (int j = 1; j < num - 1; j++) {
            grid[i][j] = 1;
        }
    }
    clearPath(); 
}

// ------------------------------------------------------- Bresenham's line algorithm for smooth drawing -----------------------------------------------
void drawLine(int x1, int y1, int x2, int y2, int value) {
    int dx = std::abs(x2 - x1);
    int dy = std::abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        if (x1 >= 0 && x1 < num && y1 >= 0 && y1 < num) {
            grid[y1][x1] = value;
        }

        if (x1 == x2 && y1 == y2) break;
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }
}

// -----------------------------------------------------------Function to create a maze using Recursive Backtracking algorithm -------------------------
void createMaze() {
    clearBoard();
    
    // Set all cells as walls initially
    for (int i = 0; i < num; i++) {
        for (int j = 0; j < num; j++) {
            grid[i][j] = 0;
        }
    }

    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 3);

    std::vector<std::pair<int, int>> stack;
    int startX = 1 + dis(gen) % (num - 2);
    int startY = 1 + dis(gen) % (num - 2);
    grid[startX][startY] = 1;
    stack.push_back(std::make_pair(startX, startY));

    while (!stack.empty()) {
        int x = stack.back().first;
        int y = stack.back().second;
        std::vector<std::pair<int, int>> neighbors;

        // Check neighbors (only vertical and horizontal)
        int dx[] = {0, 0, 1, -1};
        int dy[] = {1, -1, 0, 0};

        for (int i = 0; i < 4; i++) {
            int nx = x + dx[i] * 2;
            int ny = y + dy[i] * 2;
            if (nx > 0 && nx < num - 1 && ny > 0 && ny < num - 1 && grid[nx][ny] == 0) {
                neighbors.push_back(std::make_pair(nx, ny));
            }
        }

        if (!neighbors.empty()) {
            int index = dis(gen) % neighbors.size();
            int nx = neighbors[index].first;
            int ny = neighbors[index].second;
            grid[nx][ny] = 1;
            grid[x + (nx - x) / 2][y + (ny - y) / 2] = 1;
            stack.push_back(std::make_pair(nx, ny));
        } else {
            stack.pop_back();
        }
    }
}

#endif // MAZE_H
