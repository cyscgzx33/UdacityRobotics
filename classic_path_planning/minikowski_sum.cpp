#include <iostream>
#include <vector>
#include <set>
#include <algorithm>

using namespace std;

// Print 2D vectors coordinate values
void print2DVector(vector<vector<int> > vec)
{
     // Sorting the vector for grading purpose
    sort(vec.begin(), vec.end());
    for (int i = 0; i < vec.size(); ++i) {
        for (int j = 0; j < vec[0].size(); ++j) {
                cout << vec[i][j] << "  ";
        }
        cout << endl;
    }
}

// *** Check for duplicate coordinates inside a 2D vector and delete them*** //
vector<vector<int> > delete_duplicate(vector<vector<int> > C)
{
    // Method 0: manually implement set
    /*set<vector<int>> s;
    
    for(int i = 0; i < C.size(); i++)
    {
        s.insert(C[i]);
    }
    C.assign( s.begin(), s.end());*/
    
    
    // Method 1:  using vector sort function
    /*sort( C.begin(), C.end() );
    C.erase( unique( C.begin(), C.end() ), C.end() );*/

    // Method 2: using set constructor
    set<vector<int>> s( C.begin(), C.end());
    C.assign( s.begin(), s.end() );
    
    
    return C;
    
}

// *** Compute the Minkowski Sum of two vectors***//
vector<vector<int> > minkowski_sum(vector<vector<int> > A, vector<vector<int> > B)
{
    unsigned size_A = A.size();
    unsigned size_B = B.size();
    
    vector<vector<int>> C;
    
    for (int i = 0; i < size_A; i++)
    {
        for (int j = 0; j < size_B; j++)
        {
            C.push_back({A[i][0] + B[j][0], A[i][1] + B[j][1]});
        }
    }
    
    
    C = delete_duplicate(C);
    return C;
}

int main()
{
    // *** Define the coordinates of triangle A and B using 2D vectors*** //
    vector<vector<int>> A(3, vector<int>(2, 0));
    vector<vector<int>> B(3, vector<int>(2, 0));
    A = {{0, 1}, {1, 0}, {0, -1}};
    B = {{0, 0}, {1, 1}, {1, -1}};
    
    // Compute the minkowski sum of triangle A and B
    vector<vector<int> > C;
    C = minkowski_sum(A, B);

    // Print the resulting vector
    print2DVector(C);

    return 0;
}
