#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <climits>
#include <algorithm>
#include <string>
#include <chrono>
#include <windows.h>
#include <psapi.h>

using namespace std;

#define GRID_FILE "grid.txt"

struct Coordinate { int x, y; };
struct Node { int id; };

long long tiles_opened = 0;
long long total_weight = 0;

bool readGrid(const string &filename, int &X, int &Y, vector<vector<int>> &grid, int &start_id, int &goal_id)
{
    ifstream gridFile(filename);
    if (!gridFile.is_open()) return false;
    gridFile >> X >> Y;
    grid.assign(Y, vector<int>(X));
    start_id = -1;
    goal_id = -1;
    auto id = [X](int x, int y) { return y * X + x; };

    for (int y = 0; y < Y; ++y)
    {
        for (int x = 0; x < X; ++x)
        {
            string t;
            gridFile >> t;
            if (t == "S") { grid[y][x] = 1; start_id = id(x, y); }
            else if (t == "G") { grid[y][x] = 1; goal_id = id(x, y); }
            else grid[y][x] = atoi(t.c_str());
        }
    }
    gridFile.close();
    return true;
}

vector<Coordinate> bfs(const vector<vector<int>> &grid, int X, int Y, int start_id, int goal_id)
{
    int V = X * Y;
    vector<int> from(V, -1);
    vector<char> visited(V, 0);
    auto id = [X](int x, int y) { return y * X + x; };
    auto neighbors = [&](int node)
    {
        int x = node % X, y = node / X;
        vector<int> n;
        if (y > 0) n.push_back(id(x, y - 1));
        if (x > 0) n.push_back(id(x - 1, y));
        if (y + 1 < Y) n.push_back(id(x, y + 1));
        if (x + 1 < X) n.push_back(id(x + 1, y));
        return n;
    };

    queue<Node> q;
    q.push({start_id});
    visited[start_id] = 1;
    tiles_opened++;

    while (!q.empty())
    {
        Node cur = q.front();
        q.pop();
        if (cur.id == goal_id) break;
        for (int v : neighbors(cur.id))
        {
            int vx = v % X, vy = v / X;
            if (grid[vy][vx] == 5) continue; // obstacle
            if (visited[v]) continue;
            visited[v] = 1;
            tiles_opened++;
            from[v] = cur.id;
            q.push({v});
        }
    }

    vector<Coordinate> path;
    if (from[goal_id] != -1 || start_id == goal_id)
    {
        for (int cur = goal_id; cur != -1; cur = from[cur])
        {
            int x = cur % X, y = cur / X;
            int w = 0;
            if (grid[y][x] == 1) w = 2;
            else if (grid[y][x] == 2) w = 1;
            else if (grid[y][x] == 3) w = 3;
            else if (grid[y][x] == 4) w = 5;
            total_weight += w;
            path.push_back({x, y});
        }
        reverse(path.begin(), path.end());
    }
    return path;
}

void printMemoryUsage() {
    PROCESS_MEMORY_COUNTERS_EX pmc;
    GetProcessMemoryInfo(GetCurrentProcess(), (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof(pmc));
    SIZE_T memUsed = pmc.WorkingSetSize; 
    cout << "Memory usage: " << memUsed / (1024.0 * 1024.0) << " MB\n";
}

int main() {
    auto start = chrono::high_resolution_clock::now();

    int X, Y;
    vector<vector<int>> grid;
    int start_id, goal_id;

    if (!readGrid(GRID_FILE, X, Y, grid, start_id, goal_id))
        return 1;

    if (start_id == -1 || goal_id == -1)
    {
        cerr << "Start or Goal not found.\n";
        return 1;
    }

    auto path = bfs(grid, X, Y, start_id, goal_id);

    cout << "Shortest path (BFS):\n";
    for (auto &c : path)
    cout << c.x << "," << c.y << ":";
    cout << "\nTotal steps: " << path.size() - 1 << "\n";
    cout << "Total nodes opened: " << tiles_opened << "\n";
    cout << "Total weight: " << total_weight << "\n";
    
    auto end = chrono::high_resolution_clock::now();
    chrono::duration<double> duration = end - start;
    cout << "Execution time: " << duration.count() << " seconds\n";
    printMemoryUsage();

    return 0;
}
