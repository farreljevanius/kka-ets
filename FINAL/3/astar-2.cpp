#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <climits>
#include <algorithm>
#include <string>
#include <cmath>
#include <functional>
#include <chrono>
#include <windows.h>
#include <psapi.h>

using namespace std;

#define GRID_FILE "grid.txt"
constexpr int POKEMON_NUM = 3;

const bool COLLECT_POKEMON = true;
// Jika SIMPLE_HEURISTIC = true, maka A* Euclidean/Simple.
// Jika SIMPLE_HEURISTIC = false, maka A* Dijkstra/Complex.
const bool SIMPLE_HEURISTIC = false;

struct Coordinate
{
    int x, y;
};
struct Tunnel
{
    Coordinate A, B;
};

Tunnel tunnels[2];

struct PQItem
{
    long long f;
    long long g;
    int id;
    bool operator<(PQItem const &other) const
    {
        if (f != other.f)
            return f > other.f;
        return g < other.g;
    }
};

inline long long cellCost(int cellVal)
{
    const long long INF = LLONG_MAX / 4;
    if (cellVal == 5)
        return INF;
    switch (cellVal)
    {
    case 1:
        return 2;
    case 2:
        return 1;
    case 3:
        return 3;
    case 4:
        return 5;
    default:
        return 1;
    }
}

bool readGrid(const string &filename, int &X, int &Y, vector<vector<int>> &grid, int &start_id, int &goal_id)
{
    ifstream gridFile(filename);
    if (!gridFile.is_open())
        return false;
    gridFile >> X >> Y;
    grid.assign(Y, vector<int>(X));
    start_id = -1;
    goal_id = -1;
    auto id = [X](int x, int y)
    { return y * X + x; };
    for (int y = 0; y < Y; ++y)
        for (int x = 0; x < X; ++x)
        {
            string t;
            gridFile >> t;
            if (t == "S")
            {
                grid[y][x] = 1;
                start_id = id(x, y);
            }
            else if (t == "G")
            {
                grid[y][x] = 1;
                goal_id = id(x, y);
            }
            else if (t == "P")
            {
                grid[y][x] = 1;
            }
            else
            {
                grid[y][x] = stoi(t);
            }
        }
    gridFile.close();
    return true;
}

vector<long long> computeHeuristicDijkstra(const vector<vector<int>> &grid, int X, int Y, int goal_id)
{
    int V = X * Y;
    const long long INF = LLONG_MAX / 4;
    vector<long long> dist(V, INF);
    if (goal_id < 0)
        return dist;
    int gx = goal_id % X, gy = goal_id / X;
    if (cellCost(grid[gy][gx]) == INF)
        return dist;
    auto id = [X](int x, int y)
    { return y * X + x; };
    using PLL = pair<long long, int>;
    priority_queue<PLL, vector<PLL>, greater<PLL>> pq;
    dist[goal_id] = 0;
    pq.push({0, goal_id});
    while (!pq.empty())
    {
        auto cur = pq.top();
        pq.pop();
        long long d = cur.first;
        int u = cur.second;
        if (d != dist[u])
            continue;
        int ux = u % X, uy = u / X;
        if (uy > 0)
        {
            int v = id(ux, uy - 1);
            long long w = cellCost(grid[uy - 1][ux]);
            if (w != INF && dist[v] > dist[u] + w)
            {
                dist[v] = dist[u] + w;
                pq.push({dist[v], v});
            }
        }
        if (ux > 0)
        {
            int v = id(ux - 1, uy);
            long long w = cellCost(grid[uy][ux - 1]);
            if (w != INF && dist[v] > dist[u] + w)
            {
                dist[v] = dist[u] + w;
                pq.push({dist[v], v});
            }
        }
        if (uy + 1 < Y)
        {
            int v = id(ux, uy + 1);
            long long w = cellCost(grid[uy + 1][ux]);
            if (w != INF && dist[v] > dist[u] + w)
            {
                dist[v] = dist[u] + w;
                pq.push({dist[v], v});
            }
        }
        if (ux + 1 < X)
        {
            int v = id(ux + 1, uy);
            long long w = cellCost(grid[uy][ux + 1]);
            if (w != INF && dist[v] > dist[u] + w)
            {
                dist[v] = dist[u] + w;
                pq.push({dist[v], v});
            }
        }
        for (int ti = 0; ti < 2; ++ti)
        {
            Coordinate A = tunnels[ti].A;
            Coordinate B = tunnels[ti].B;
            int a_id = id(A.x, A.y);
            int b_id = id(B.x, B.y);
            if (u == a_id)
            {
                long long w = cellCost(grid[B.y][B.x]);
                if (w != INF && dist[b_id] > dist[u] + w)
                {
                    dist[b_id] = dist[u] + w;
                    pq.push({dist[b_id], b_id});
                }
            }
            else if (u == b_id)
            {
                long long w = cellCost(grid[A.y][A.x]);
                if (w != INF && dist[a_id] > dist[u] + w)
                {
                    dist[a_id] = dist[u] + w;
                    pq.push({dist[a_id], a_id});
                }
            }
        }
    }
    return dist;
}

vector<long long> computeHeuristicEuclidean(const vector<vector<int>> &grid, int width, int height, int goalId)
{
    const long long INF = LLONG_MAX / 4;
    vector<long long> heuristic(width * height, INF);
    if (goalId < 0)
        return heuristic;
    int goalX = goalId % width;
    int goalY = goalId / width;
    if (cellCost(grid[goalY][goalX]) == INF)
        return heuristic;
    long long minCost = LLONG_MAX;
    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x)
        {
            long long c = cellCost(grid[y][x]);
            if (c < minCost)
                minCost = c;
        }
    if (minCost <= 0 || minCost >= LLONG_MAX / 4)
        minCost = 1;
    auto index = [width](int x, int y)
    { return y * width + x; };
    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x)
        {
            long long c = cellCost(grid[y][x]);
            if (c == INF)
                continue;
            double dx = x - goalX;
            double dy = y - goalY;
            double dist = sqrt(dx * dx + dy * dy);
            long long scaled = static_cast<long long>(ceil(dist * double(minCost)));
            heuristic[index(x, y)] = scaled;
        }
    return heuristic;
}

long long total_nodes_opened = 0;

vector<Coordinate> astar(const vector<vector<int>> &grid, const vector<long long> &heuristic, int X, int Y, int start_id, int goal_id)
{
    int V = X * Y;
    const long long INF = LLONG_MAX / 4;
    vector<long long> g(V, INF), f(V, INF);
    vector<int> from(V, -1);
    vector<char> closed(V, 0);
    priority_queue<PQItem> open;
    auto id = [X](int x, int y)
    { return y * X + x; };
    auto neighbors = [&](int node)
    {
        int x = node % X, y = node / X;
        vector<int> n;
        if (y > 0)
            n.push_back(id(x, y - 1));
        if (x > 0)
            n.push_back(id(x - 1, y));
        if (y + 1 < Y)
            n.push_back(id(x, y + 1));
        if (x + 1 < X)
            n.push_back(id(x + 1, y));
        for (int ti = 0; ti < 2; ++ti)
        {
            Coordinate A = tunnels[ti].A;
            Coordinate B = tunnels[ti].B;
            int a_id = id(A.x, A.y);
            int b_id = id(B.x, B.y);
            if (node == a_id)
                n.push_back(b_id);
            else if (node == b_id)
                n.push_back(a_id);
        }
        return n;
    };
    g[start_id] = 0;
    f[start_id] = (heuristic[start_id] == INF) ? 0 : heuristic[start_id];
    open.push({f[start_id], g[start_id], start_id});
    while (!open.empty())
    {
        auto cur = open.top();
        open.pop();
        int u = cur.id;
        if (closed[u])
        {
            if (cur.g > g[u])
                continue;
        }
        if (u == goal_id)
        {
            closed[u] = 1;
            total_nodes_opened++;
            break;
        }
        if (closed[u] == 0)
        {
            closed[u] = 1;
            total_nodes_opened++;
        }
        for (int v : neighbors(u))
        {
            int vx = v % X, vy = v / X;
            long long w = cellCost(grid[vy][vx]);
            if (w == INF)
                continue;
            if (g[u] == INF)
                continue;
            long long tentative = g[u] + w;
            if (tentative < g[v])
            {
                from[v] = u;
                g[v] = tentative;
                long long h = (heuristic[v] == INF) ? 0 : heuristic[v];
                f[v] = tentative + h;
                open.push({f[v], g[v], v});
                if (closed[v])
                    closed[v] = 0;
            }
        }
    }
    vector<Coordinate> path;
    if (from[goal_id] != -1 || start_id == goal_id)
    {
        for (int cur = goal_id; cur != -1; cur = from[cur])
            path.push_back({cur % X, cur / X});
        reverse(path.begin(), path.end());
    }
    return path;
}

void printPath(const vector<Coordinate> &path)
{
    for (auto &c : path)
        cout << c.x << "," << c.y << ":";
}

inline int getID(int X, int x, int y) { return y * X + x; }
inline float eucledian(int x1, int y1, int x2, int y2) { return sqrtf(float(x1 - x2) * float(x1 - x2) + float(y1 - y2) * float(y1 - y2)); }

void printMemoryUsage() {
    PROCESS_MEMORY_COUNTERS_EX pmc;
    GetProcessMemoryInfo(GetCurrentProcess(), (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof(pmc));
    SIZE_T memUsed = pmc.WorkingSetSize;
    cout << "Memory usage: " << memUsed / (1024.0 * 1024.0) << " MB\n";
}

int main()
{
    auto start = chrono::high_resolution_clock::now();

    tunnels[0] = {{6, 2}, {3, 14}};
    tunnels[1] = {{10, 5}, {10, 16}};

    long long total_steps = 0;
    long long total_weight = 0;

    if (!COLLECT_POKEMON)
    {
        int X, Y;
        vector<vector<int>> grid;
        int start_id, goal_id;
        if (!readGrid(GRID_FILE, X, Y, grid, start_id, goal_id))
            return 1;
        int V = X * Y;
        if (start_id == -1)
            start_id = 0;
        if (goal_id == -1)
            goal_id = V - 1;
        vector<long long> heuristic;
        if (SIMPLE_HEURISTIC)
            heuristic = computeHeuristicEuclidean(grid, X, Y, goal_id);
        else
            heuristic = computeHeuristicDijkstra(grid, X, Y, goal_id);
        vector<Coordinate> path = astar(grid, heuristic, X, Y, start_id, goal_id);
        printPath(path);
        if (!path.empty())
        {
            long long steps_total = static_cast<long long>(path.size()) - 1;
            long long weight_total = 0;
            for (size_t i = 1; i < path.size(); ++i)
            {
                int px = path[i].x;
                int py = path[i].y;
                long long w = cellCost(grid[py][px]);
                if (w < LLONG_MAX / 4)
                    weight_total += w;
            }
            total_steps += steps_total;
            total_weight += weight_total;
        }
        cout << "\nTotal steps: " << total_steps;
        cout << "\nTotal nodes opened: " << total_nodes_opened;
        cout << "\nTotal weight: " << total_weight;

        auto end = chrono::high_resolution_clock::now();
        chrono::duration<double> duration = end - start;
        cout << "Execution time: " << duration.count() << " seconds\n";

        printMemoryUsage();
        return 0;
    }
    else
    {
        ifstream gridFile(GRID_FILE);
        if (!gridFile.is_open())
            return 1;
        int X, Y;
        gridFile >> X >> Y;
        vector<vector<int>> grid(Y, vector<int>(X));
        int start_id = -1, goal_id = -1;
        auto id = [X](int x, int y)
        { return y * X + x; };
        Coordinate pokemons[POKEMON_NUM];
        int collected_pokemons[POKEMON_NUM];
        int poke_i = 0;
        for (int i = 0; i < POKEMON_NUM; i++)
            collected_pokemons[i] = 0;
        for (int y = 0; y < Y; ++y)
            for (int x = 0; x < X; ++x)
            {
                string t;
                gridFile >> t;
                if (t == "S")
                {
                    grid[y][x] = 1;
                    start_id = id(x, y);
                }
                else if (t == "G")
                {
                    grid[y][x] = 1;
                    goal_id = id(x, y);
                }
                else if (t == "P")
                {
                    grid[y][x] = 1;
                    Coordinate poke_cor = {x, y};
                    if (poke_i < POKEMON_NUM)
                        pokemons[poke_i++] = poke_cor;
                }
                else
                {
                    grid[y][x] = stoi(t);
                }
            }
        gridFile.close();
        int V = X * Y;
        if (start_id == -1)
            start_id = 0;
        if (goal_id == -1)
            goal_id = V - 1;
        vector<long long> heuristic;
        if (SIMPLE_HEURISTIC)
            heuristic = computeHeuristicEuclidean(grid, X, Y, goal_id);
        else
            heuristic = computeHeuristicDijkstra(grid, X, Y, goal_id);
        Coordinate current;
        cout << "Shortest path visiting all Pokemons (A*):\n";
        current = {start_id % X, start_id / X};
        cout << current.x << "," << current.y << ":";
        for (int i = 0; i < POKEMON_NUM; i++)
        {
            Coordinate closest_pokemon;
            int closest_pokemon_id = -1;
            float closest_pokemon_dist = -1.0;
            for (int j = 0; j < poke_i; j++)
            {
                if (collected_pokemons[j] == 0)
                {
                    if (closest_pokemon_dist < 0)
                    {
                        closest_pokemon = pokemons[j];
                        closest_pokemon_dist = eucledian(current.x, current.y, pokemons[j].x, pokemons[j].y);
                        closest_pokemon_id = j;
                    }
                    else
                    {
                        float d = eucledian(current.x, current.y, pokemons[j].x, pokemons[j].y);
                        if (d < closest_pokemon_dist)
                        {
                            closest_pokemon = pokemons[j];
                            closest_pokemon_dist = d;
                            closest_pokemon_id = j;
                        }
                    }
                }
            }
            if (closest_pokemon_id == -1)
                break;
            vector<Coordinate> path = astar(grid, heuristic, X, Y, getID(X, current.x, current.y), getID(X, closest_pokemon.x, closest_pokemon.y));
            for (auto &c : path)
                cout << c.x << "," << c.y << ":";
            if (!path.empty())
            {
                long long steps_total = static_cast<long long>(path.size()) - 1;
                long long weight_total = 0;
                for (size_t ii = 1; ii < path.size(); ++ii)
                {
                    int px = path[ii].x;
                    int py = path[ii].y;
                    long long w = cellCost(grid[py][px]);
                    if (w < LLONG_MAX / 4)
                        weight_total += w;
                }
                total_steps += steps_total;
                total_weight += weight_total;
            }
            if (!path.empty())
                current = path.back();
            else
            {
                collected_pokemons[closest_pokemon_id] = 1;
                continue;
            }
            collected_pokemons[closest_pokemon_id] = 1;
        }
        vector<Coordinate> path = astar(grid, heuristic, X, Y, getID(X, current.x, current.y), goal_id);
        for (auto &c : path)
            cout << c.x << "," << c.y << ":";
        if (!path.empty())
        {
            long long steps_total = static_cast<long long>(path.size()) - 1;
            long long weight_total = 0;
            for (size_t ii = 1; ii < path.size(); ++ii)
            {
                int px = path[ii].x;
                int py = path[ii].y;
                long long w = cellCost(grid[py][px]);
                if (w < LLONG_MAX / 4)
                    weight_total += w;
            }
            total_steps += steps_total;
            total_weight += weight_total;
        }
        cout << "\nTotal steps: " << total_steps;
        cout << "\nTotal nodes opened: " << total_nodes_opened;
        cout << "\nTotal weight: " << total_weight;

        auto end = chrono::high_resolution_clock::now();
        chrono::duration<double> duration = end - start;
        cout << "\nExecution time: " << duration.count() << " seconds\n";

        printMemoryUsage();
        return 0;
    }
}


