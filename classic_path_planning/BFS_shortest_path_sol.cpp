#include <iostream>
#include <string.h>
#include <vector>
#include <algorithm>

using namespace std;

// Map class
class Map {
public:
    const static int mapWidth = 6;
    const static int mapHeight = 5;
    vector<vector<int> > grid = {
        { 0, 1, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0 },
        { 0, 1, 0, 0, 0, 0 },
        { 0, 0, 0, 1, 1, 0 }
    };
};

// Planner class
class Planner : Map {
public:
    int start[2] = { 0, 0 };
    int goal[2] = { mapHeight - 1, mapWidth - 1 };
    int cost = 1;

    string movements_arrows[4] = { "^", "<", "v", ">" };

    vector<vector<int> > movements{
        { -1, 0 },
        { 0, -1 },
        { 1, 0 },
        { 0, 1 }
    };
};

// Template function to print 2D vectors of any type
template <typename T>
void print2DVector(T Vec)
{
    for (int i = 0; i < Vec.size(); ++i) {
        for (int j = 0; j < Vec[0].size(); ++j) {
            cout << Vec[i][j] << ' ';
        }
        cout << endl;
    }
}

// Search function will generate the expansions
void search(Map map, Planner planner)
{
    // Create a closed 2 array filled with 0s and first element 1
    vector<vector<int> > closed(map.mapHeight, vector<int>(map.mapWidth));
    closed[planner.start[0]][planner.start[1]] = 1;

    // Create expand array filled with -1
    vector<vector<int> > expand(map.mapHeight, vector<int>(map.mapWidth, -1));

    // Create action array filled with -1
    vector<vector<int> > action(map.mapHeight, vector<int>(map.mapWidth, -1));

    // Defined the triplet values
    int x = planner.start[0];
    int y = planner.start[1];
    int g = 0;

    // Store the expansions
    vector<vector<int> > open;
    open.push_back({ g, x, y });

    // Flags and counters
    bool found = false;
    bool resign = false;
    int count = 0;

    int x2;
    int y2;

    // While I am still searching for the goal and the problem is solvable
    while (!found && !resign) {
        // Resign if no values in the open list and you can't expand anymore
        if (open.size() == 0) {
            resign = true;
            cout << "Failed to reach a goal" << endl;
        }
        // Keep expanding
        else {
            // Remove triplets from the open list
            sort(open.begin(), open.end());
            reverse(open.begin(), open.end());
            vector<int> next;
            // Stored the poped value into next
            next = open.back();
            open.pop_back();

            x = next[1];
            y = next[2];
            g = next[0];

            // Fill the expand vectors with count
            expand[x][y] = count;
            count += 1;

            // Check if we reached the goal:
            if (x == planner.goal[0] && y == planner.goal[1]) {
                found = true;
                //cout << "[" << g << ", " << x << ", " << y << "]" << endl;
            }

            //else expand new elements
            else {
                for (int i = 0; i < planner.movements.size(); i++) {
                    x2 = x + planner.movements[i][0];
                    y2 = y + planner.movements[i][1];
                    if (x2 >= 0 && x2 < map.grid.size() && y2 >= 0 && y2 < map.grid[0].size()) {
                        if (closed[x2][y2] == 0 and map.grid[x2][y2] == 0) {
                            int g2 = g + planner.cost;
                            open.push_back({ g2, x2, y2 });
                            closed[x2][y2] = 1;
                            action[x2][y2] = i;
                        }
                    }
                }
            }
        }
    }
    // Print the expansion List
    //print2DVector(expand);

    // Find the path with robot orientation
    vector<vector<string> > policy(map.mapHeight, vector<string>(map.mapWidth, "-"));

    // Going backward
    x = planner.goal[0];
    y = planner.goal[1];
    policy[x][y] = '*';

    while (x != planner.start[0] or y != planner.start[1]) {
        x2 = x - planner.movements[action[x][y]][0];
        y2 = y - planner.movements[action[x][y]][1];
        policy[x2][y2] = planner.movements_arrows[action[x][y]];
        x = x2;
        y = y2;
    }

    // Print the path with arrows
    print2DVector(policy);
}

int main()
{
    // Instantiate map and planner objects
    Map map;
    Planner planner;

    // Search for the expansions
    search(map, planner);

    return 0;
}

