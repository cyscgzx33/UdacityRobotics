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
struct myCompare {
    bool operator()(vector<int> const & a, vector<int> const & b)
    { 
      int aCount = 0, bCount = 0;
      if (a[0] > b[0]) aCount += 2;
      else if (a[0] < b[0]) bCount += 2;
      if (a[2] > b[2]) aCount += 1;
      else bCount += 1;

      return aCount > bCount;
    }
};




void search(Map map, Planner planner)
{
    priority_queue<vector<int>, vector<vector<int>>, myCompare> q;
    vector<vector<int>> expansion(map.mapHeight, vector<int>(map.mapWidth, -1));
    vector<vector<bool>> isVisited(map.mapHeight, vector<bool>(map.mapWidth, false));

    int steps = 0;

    int startRow = planner.start[0];
    int startCol = planner.start[1];

    int startPos = startRow * map.mapWidth + startCol;
    int startCost = map.grid[startRow][startCol];
    q.push({startCost, startPos, steps});

    

    while(!q.empty())
    {

      int curRow = q.top()[1] / map.mapWidth;
      int curCol = q.top()[1] % map.mapWidth;
      q.pop();

      // print out to check
      /*cout << "row = "     << curRow     << ", " 
           << "col = "     << curCol     << ", " 
           << "nextPos = " << q.top()[1] << ", " 
           << "steps = "   << q.top()[2] << endl;*/
      
      isVisited[curRow][curCol] = true;
      expansion[curRow][curCol] = steps;

      if (curRow == planner.goal[0] && curCol == planner.goal[1]) break;

      for (auto ele : planner.movements)
      {
          int nextRow = curRow + ele[0];
          int nextCol = curCol + ele[1];
          
          if (nextRow >= 0 && nextRow < map.mapHeight && nextCol >= 0 && nextCol < map.mapWidth && !isVisited[nextRow][nextCol])
          {
            // isVisited should also exist in this function
            isVisited[nextRow][nextCol] = true;
            int nextPos = nextRow * map.mapWidth + nextCol;
            int nextCost = map.grid[nextRow][nextCol];
            q.push({nextCost, nextPos, steps});
          }
      }

      steps++;
      
    }

    print2DVector(expansion);
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
