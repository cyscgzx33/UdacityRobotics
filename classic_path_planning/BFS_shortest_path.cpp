#include <iostream>
#include <string.h>
#include <vector>
#include <algorithm>
#include <queue>

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



/*#### Implement the search function which will generate the expansion list ####*/
// You are only required to print the final triplet values


// Keep an eye on this compare function


// Method 1: seems working
struct myCompare {
    bool operator()(vector<int> const & a, vector<int> const & b)
    { 
      /* Priority I: cost of each cell, most important */
      if (a[0] > b[0]) return true;
      else if (a[0] < b[0]) return false;
      else 
      {
        /* Priority II: cost of distance, intermediatly important */
        if (a[4] > b[4]) return true;
        else if (a[4] < b[4])return false; // no priority IV, so stop here
        else
        {
          /* Priority III: cost of steps, least important */
          if (a[2] > b[2]) return true;
          else return false;
        }
      }
    }
};

// Method 1: seems working, as well
// struct myCompare {
//   bool operator() (vector<int> const& a, vector<int> const& b)
//   {
//     int g_a = 0, g_b = 0;
//     int h_a = 0, h_b = 0;

//     g_a = a[0] * 1000;
//     g_b = b[0] * 1000;
//     h_a = a[4] * 10;
//     h_b = b[4] * 10;

//     return g_a + h_a > g_b + h_b;
//   }
// };



void search(Map map, Planner planner)
{
    priority_queue<vector<int>, vector<vector<int>>, myCompare> q;
    vector<vector<int>> expansion(map.mapHeight, vector<int>(map.mapWidth, -1));
    vector<vector<bool>> isVisited(map.mapHeight, vector<bool>(map.mapWidth, false));
    vector<vector<char>> direction(map.mapHeight, vector<char>(map.mapWidth, '-'));

    int steps = 0;

    int startRow = planner.start[0];
    int startCol = planner.start[1];
    int goalRow  = planner.goal[0];
    int goalCol  = planner.goal[1];

    int startPos = startRow * map.mapWidth + startCol;
    int startCost = map.grid[startRow][startCol];

    // modified to implement nextDir
    int startDir = -1; // -1 means unsure yet

    // modified to implement startDist
    int startDist = ( goalRow - startRow ) + ( goalCol - startCol );

    // added two additional elements: startDir & startDist
    q.push({startCost, startPos, steps, startDir, startDist});


    while(!q.empty())
    {
      int curRow = q.top()[1] / map.mapWidth;
      int curCol = q.top()[1] % map.mapWidth;
      q.pop();

      // print out to check
      cout << "row = "     << curRow     << ", " 
           << "col = "     << curCol     << ", " 
           << "nextPos = " << q.top()[1] << ", " 
           << "steps = "   << q.top()[2] << endl;
      
      isVisited[curRow][curCol] = true;
      expansion[curRow][curCol] = steps;

      if (curRow == planner.goal[0] && curCol == planner.goal[1]) 
      {
        direction[curRow][curCol] = '*';
        break;
      }
      for (int i = 0; i < planner.movements.size(); i++)
      {
          int nextRow = curRow + planner.movements[i][0];
          int nextCol = curCol + planner.movements[i][1];
          
          if (nextRow >= 0 && nextRow < map.mapHeight && nextCol >= 0 && nextCol < map.mapWidth && !isVisited[nextRow][nextCol])
          {
            // isVisited should also exist in this function
            isVisited[nextRow][nextCol] = true;
            int nextPos = nextRow * map.mapWidth + nextCol;
            int nextCost = map.grid[nextRow][nextCol];
            int nextDist = ( goalRow - nextRow ) + ( goalCol - nextCol );
            q.push({nextCost, nextPos, steps, i, nextDist}); // modified to implement nextDir
          }
      }

      // added a part to decide what is direction of the next step
      char nextDir = planner.movements_arrows[q.top()[3]][0]; // Trick: convert from string to a single char
      direction[curRow][curCol] = nextDir;

      steps++;
      
    }

    print2DVector(direction);
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
