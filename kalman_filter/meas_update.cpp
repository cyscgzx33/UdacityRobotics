#include <iostream>
#include <math.h>
#include <tuple>

using namespace std;

double new_mean, new_var;

tuple<double, double> measurement_update(double mean1, double var1, double mean2, double var2)
{
    new_mean = (mean1/var1 + mean2/var2) / (1/var1 + 1/var2);//TODO: Code the measurment update mean function mu;
    new_var =  1 / (1/var1 + 1/var2);//TODO: Code the measurment update variance function sigma square;
    return make_tuple(new_mean, new_var);
}

int main()
{
    // it seems "tuple" and "tie" are usually showing up together
    tie(new_mean, new_var) = measurement_update(10, 8, 13, 2);
    printf("[%f, %f]", new_mean, new_var);
    return 0;
}
