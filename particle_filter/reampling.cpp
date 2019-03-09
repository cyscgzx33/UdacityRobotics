#include <iostream>
#include <stdio.h>
#include <vector>
using namespace std;

double w[] = { 0.6, 1.2, 2.4, 0.6, 1.2 };//You can also change this to a vector
vector<double> w_vec(w, w+5);
//TODO: Define a  ComputeProb function and compute the Probabilities
double ComputeProb(vector<double> w_vec, double tar) 
{
    double sum = 0;
    // is "auto" not allowed in c++98?
    // for (auto& a : w_vec) sum += a; 
    for (int i = 0 ; i < w_vec.size(); i++) sum += w_vec[i];
    
    return tar / sum;
}


int main()
{
    //TODO: Print Probabilites each on a single line:
    //P1=Value
    //:
    //P5=Value
    printf("P1=%f\n", ComputeProb(w_vec, w_vec[0]));
    printf("P2=%f\n", ComputeProb(w_vec, w_vec[1]));
    printf("P3=%f\n", ComputeProb(w_vec, w_vec[2]));
    printf("P4=%f\n", ComputeProb(w_vec, w_vec[3]));
    printf("P5=%f\n", ComputeProb(w_vec, w_vec[4]));
    
    
    
    return 0;
}
