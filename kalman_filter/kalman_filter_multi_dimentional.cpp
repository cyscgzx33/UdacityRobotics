#include <iostream>
#include <math.h>
#include <tuple>
#include "Core" // Eigen Library
#include "LU"   // Eigen Library

using namespace std;
using namespace Eigen;

float measurements[3] = { 1, 2, 3 };

tuple<MatrixXf, MatrixXf> kalman_filter(MatrixXf x, MatrixXf P, MatrixXf u, MatrixXf F, MatrixXf H, MatrixXf R, MatrixXf I)
{
    for (int n = 0; n < sizeof(measurements) / sizeof(measurements[0]); n++) {
        //****** TODO: Kalman-filter function********//
        
        // Measurement Update
        // Initialize and Compute Z, y, S, K, x, and P
        MatrixXf z(1, 1);
        z << measurements[n]; // measurement
        MatrixXf y(1, 1); 
        y << z - H * x;
        MatrixXf S(1, 1); 
        S << H * P * H.transpose() + R;
        
        // Calculation of Kalman Gain
        MatrixXf K(2, 1);
        K << P * H.transpose() * S.inverse();
        
        // Calculation of Posterior State and Covariance
        x << x + K * y;
        P << (I - K * H) * P;
        
        // State Prediction
        x << F * x;
        P << F * P * F.transpose(); // Assume Q = 0
                
        
    }

    return make_tuple(x, P);
}

int main()
{

    MatrixXf x(2, 1);// Initial state (location and velocity) 
    x << 0,
    	 0; 
    MatrixXf P(2, 2);//Initial Uncertainty
    P << 100, 0, 
    	 0, 100; 
    MatrixXf u(2, 1);// External Motion
    u << 0,
    	 0; 
    MatrixXf F(2, 2);//Next State Function
    F << 1, 1,
    	 0, 1; 
    MatrixXf H(1, 2);//Measurement Function
    H << 1,
    	 0; 
    MatrixXf R(1, 1); //Measurement Uncertainty
    R << 1;
    MatrixXf I(2, 2);// Identity Matrix
    I << 1, 0,
    	 0, 1; 

    tie(x, P) = kalman_filter(x, P, u, F, H, R, I);
    cout << "x= " << x << endl;
    cout << "P= " << P << endl;

    return 0;
}
