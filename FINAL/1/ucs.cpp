#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <climits>
#include <algorithm>
#include <string>
#include <cmath>
#include <chrono>
#include <windows.h>
#include <psapi.h>

using namespace std;

#define GRID_FILE "grid.txt"

struct Coordinate { int x, y; };
struct Node { int id; float cost; bool operator>(const Node &o) const { return cost > o.cost; } };

int nodes_opened = 0;

int getWeight(int t) {
    if (t == 1) return 2;
    if (t == 2) return 1;
    if (t == 3) return 3;
    if (t == 4) return 5;
    return INT_MAX;
}

bool readGrid(const string &f, int &X, int &Y, vector<vector<int>> &g, int &s, int &e) {
    ifstream in(f);
    if (!in.is_open()) return false;
    in >> X >> Y;
    g.assign(Y, vector<int>(X));
    auto id = [X](int x, int y) { return y * X + x; };
    for (int y = 0; y < Y; ++y)
        for (int x = 0; x < X; ++x) {
            string t; in >> t;
            if (t == "S") { g[y][x] = 1; s = id(x, y); }
            else if (t == "G") { g[y][x] = 1; e = id(x, y); }
            else g[y][x] = atoi(t.c_str());
        }
    return true;
}

vector<Coordinate> ucs(const vector<vector<int>> &g, int X, int Y, int s, int e, float &tc) {
    int V = X * Y;
    vector<int> from(V, -1);
    vector<float> cost(V, INFINITY);
    vector<char> vis(V, 0);
    auto id = [X](int x, int y) { return y * X + x; };
    auto nbs = [&](int n) {
        int x = n % X, y = n / X; vector<int> r;
        if (y > 0) r.push_back(id(x, y - 1));
        if (x > 0) r.push_back(id(x - 1, y));
        if (y + 1 < Y) r.push_back(id(x, y + 1));
        if (x + 1 < X) r.push_back(id(x + 1, y));
        return r;
    };
    priority_queue<Node, vector<Node>, greater<Node>> pq;
    pq.push({s, 0}); cost[s] = 0;
    while (!pq.empty()) {
        Node cur = pq.top(); pq.pop();
        if (vis[cur.id]) continue;
        vis[cur.id] = 1; nodes_opened++;
        if (cur.id == e) break;
        for (int v : nbs(cur.id)) {
            int vx = v % X, vy = v / X;
            if (g[vy][vx] == 5) continue;
            float w = getWeight(g[vy][vx]);
            if (w == INT_MAX) continue;
            float nc = cost[cur.id] + w;
            if (nc < cost[v]) { cost[v] = nc; from[v] = cur.id; pq.push({v, nc}); }
        }
    }
    tc = cost[e];
    vector<Coordinate> path;
    if (from[e] != -1 || s == e) {
        for (int cur = e; cur != -1; cur = from[cur])
            path.push_back({cur % X, cur / X});
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

    int X, Y, s = -1, e = -1;
    vector<vector<int>> g;

    if (!readGrid(GRID_FILE, X, Y, g, s, e)) return 1;
    if (s == -1 || e == -1) return 1;

    float total_cost = 0;
    auto path = ucs(g, X, Y, s, e, total_cost);
    if (path.empty()) { cout << "No path found.\n"; return 0; }

    cout << "Shortest path (UCS):\n";
    for (auto &c : path) 
    cout << c.x << "," << c.y << ":";
    cout << "\nTotal steps: " << path.size() - 1;
    cout << "\nTotal nodes opened: " << nodes_opened;
    cout << "\nTotal weight: " << total_cost << "\n";
    
    auto end = chrono::high_resolution_clock::now();
    chrono::duration<double> duration = end - start;
    cout << "Execution time: " << duration.count() << " seconds\n";
    printMemoryUsage();

    return 0;
}
