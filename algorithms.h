


// ==============================================================================================================================================================================================================
//
//                                                                                          ALGORITHMS.H
//
// ==============================================================================================================================================================================================================
//

#ifndef ALGORITHMS_H
#define ALGORITHMS_H

// ---------------------------------------------------------------------------------------------- Header files ---------------------------------------------------------------

#include <vector>
#include <queue>
#include <cmath>
#include <cfloat>
#include <chrono>

extern const int num;
extern int grid[60][60];
extern std::vector<std::pair<int, int>> path;
extern std::vector<std::pair<int, int>> exploredCells;
extern bool explored[60][60];
extern bool isPathFound;
extern std::string algorithmUsed;
extern int stepsTaken;
extern float timeTaken;
extern bool isSearching;
extern std::string statusMessage;
extern bool destinationReached;

void clearPath();

// ------------------------------------------------------------------------------Dijkstra's algorithm -------------------------------------------------------------

void dijkstra(int source_x, int source_y, int dest_x, int dest_y) {
    clearPath();
    auto startTime = std::chrono::high_resolution_clock::now();
    isSearching = true;
    
    std::priority_queue<std::pair<float, std::pair<int, int>>, std::vector<std::pair<float, std::pair<int, int>>>, std::greater<std::pair<float, std::pair<int, int>>>> pq;
    std::vector<std::vector<float>> dist(num, std::vector<float>(num, FLT_MAX));
    std::vector<std::vector<std::pair<int, int>>> previous(num, std::vector<std::pair<int, int>>(num, std::make_pair(-1, -1)));

    dist[source_x][source_y] = 0.0;
    pq.push(std::make_pair(0.0, std::make_pair(source_x, source_y)));

    while (!pq.empty()) {
        int x = pq.top().second.first;
        int y = pq.top().second.second;
        pq.pop();

        explored[x][y] = true;
        exploredCells.push_back(std::make_pair(x, y));
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
                    previous[nx][ny] = std::make_pair(x, y);
                    pq.push(std::make_pair(newDist, std::make_pair(nx, ny)));
                }
            }
        }
    }

 // ------------------------------------------------------------------------------ Re-Construct path -------------------------------------------------------------
 
    int x = dest_x, y = dest_y;
    while (x != source_x || y != source_y) {
        if (previous[x][y].first == -1 && previous[x][y].second == -1) {
            // Path not found
            statusMessage = "No path found to destination";
            isSearching = false;
            return;
        }
        path.push_back(std::make_pair(x, y));
        int px = previous[x][y].first;
        int py = previous[x][y].second;
        x = px;
        y = py;
    }
    path.push_back(std::make_pair(source_x, source_y));
    std::reverse(path.begin(), path.end());

    isPathFound = true;
    algorithmUsed = "Dijkstra";
    statusMessage = "Path found!";
}


// ------------------------------------------------------------------------------ A* algorithm -------------------------------------------------------------

void astar(int source_x, int source_y, int dest_x, int dest_y) {
    clearPath();
    auto startTime = std::chrono::high_resolution_clock::now();
    isSearching = true;
    destinationReached = false;
    
    auto heuristic = [](int x1, int y1, int x2, int y2) -> float {
        return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2)); // Euclidean distance
    };

    std::priority_queue<std::pair<float, std::pair<int, int>>, std::vector<std::pair<float, std::pair<int, int>>>, std::greater<std::pair<float, std::pair<int, int>>>> pq;
    std::vector<std::vector<float>> g_score(num, std::vector<float>(num, FLT_MAX));
    std::vector<std::vector<float>> f_score(num, std::vector<float>(num, FLT_MAX));
    std::vector<std::vector<std::pair<int, int>>> previous(num, std::vector<std::pair<int, int>>(num, std::make_pair(-1, -1)));

    g_score[source_x][source_y] = 0.0;
    f_score[source_x][source_y] = heuristic(source_x, source_y, dest_x, dest_y);
    pq.push(std::make_pair(f_score[source_x][source_y], std::make_pair(source_x, source_y)));

    while (!pq.empty()) {
        int x = pq.top().second.first;
        int y = pq.top().second.second;
        pq.pop();

        explored[x][y] = true;
        exploredCells.push_back(std::make_pair(x, y));
        stepsTaken++;

        if (x == dest_x && y == dest_y) {
            destinationReached = true;
            break;
        }

        int dx[] = {0, 0, 1, -1, 1, 1, -1, -1};
        int dy[] = {1, -1, 0, 0, 1, -1, 1, -1};

        for (int i = 0; i < 8; i++) {
            int nx = x + dx[i];
            int ny = y + dy[i];
            if (nx >= 0 && nx < num && ny >= 0 && ny < num && grid[nx][ny] == 1 && !explored[nx][ny]) {
                float tentative_g_score = g_score[x][y] + (i < 4 ? 1.0f : 1.414f);
                if (tentative_g_score < g_score[nx][ny]) {
                    previous[nx][ny] = std::make_pair(x, y);
                    g_score[nx][ny] = tentative_g_score;
                    f_score[nx][ny] = g_score[nx][ny] + heuristic(nx, ny, dest_x, dest_y);
                    pq.push(std::make_pair(f_score[nx][ny], std::make_pair(nx, ny)));
                }
            }
        }
    }

// ------------------------------------------------------------------------------ Re-Construct path -------------------------------------------------------------

    int x = dest_x, y = dest_y;
    while (x != source_x || y != source_y) {
        if (previous[x][y].first == -1 && previous[x][y].second == -1) {
            // Path not found
            statusMessage = "No path found to destination";
            isSearching = false;
            return;
        }
        path.push_back(std::make_pair(x, y));
        int px = previous[x][y].first;
        int py = previous[x][y].second;
        x = px;
        y = py;
    }
    path.push_back(std::make_pair(source_x, source_y));
    std::reverse(path.begin(), path.end());

    isPathFound = true;
    algorithmUsed = "A*";
    statusMessage = "Path found!";
    
    // Record end time and calculate total time taken
    auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> elapsedTime = endTime - startTime;
    timeTaken = elapsedTime.count();
}

#endif // ALGORITHMS_H
