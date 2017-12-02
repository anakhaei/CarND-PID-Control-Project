#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID()
{
    d_error = 0;
    p_error = 0;
    i_error = 0;
    isInitialized = false;
    //twiddleActive = false;
    p.resize(3);
    // dp.resize(3);
    // tol.resize(3);
    // dp[0]=0.1;
    // dp[1]=1;
    // dp[2]=0.01;
    // for (unsigned int i =0 ; i< 3; i++){
    //     tol[i] = dp[i]/10;
    // }
    aveError = 0;
    it = 0;
    maxIt = 100;
    sumError =0;
}

PID::~PID() {}

void PID::Init(double Kp_in, double Ki_in, double Kd_in)
{
    p[0] = Kp_in;
    p[1] = Ki_in;
    p[2]= Kd_in;
    isInitialized = false;
}

void PID::UpdateError(double cte)
{
    if (!isInitialized)
    {
        d_error = 0;
        i_error = 0;
        p_error = cte;
        isInitialized = true;
    }
    else
    {
        double aveCte = (p_error + cte)/2;
        d_error = (d_error + aveCte - p_error)/2; //smoothing the d_error
        p_error = aveCte;
        i_error += aveCte;
    }
    it++;
    sumError += std::abs(p_error);
    aveError = sumError/it;

}
double PID::TotalError(double speed)
{

    //double error = -p[0] * p_error - p[1] * d_error  - p[2] * i_error;
    double speedfactor =0;

    speedfactor = (speed +1)/10;
    double error = -p[0] * p_error / speedfactor - p[1] * d_error /speedfactor  - p[2] * i_error/ speedfactor;
    std::cout <<"it= " << it << ", speed = " << speed << "aveError= "<< aveError << ", p_error =" << p_error << " , d_error =" << d_error << ", i_error= " << i_error << ", steering = " << error << endl;
    //eturn (-Kp * p_error - Kd * d_error - Ki * i_error);
    return error;
}

// double PID::Twiddle(){
//     if (dp[0]<tol[0] && dp[1] < tol[1] && dp[2] < tol[2]){
//         cout << "Twiddle Success ------------- p[0] =" << p[0] << ", p[1]" << p[1] << ", p[2]" << p[2] << endl;
//         return TotalError(); 

//     }else{
//         if (it < maxIt) {
//             aveError += abs(p_error);
//             it++;
//             return TotalError();
//         }else {
//             aveError = 0;
//             it = 0;
//             if ()

//         }


//     }
// }
