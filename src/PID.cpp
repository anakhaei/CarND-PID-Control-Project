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
    dp.resize(3);
    tol.resize(3);
    dp[0] = 0.1;
    dp[1] = 1;
    dp[2] = 0.01;
    for (unsigned int i = 0; i < 3; i++)
    {
        tol[i] = dp[i] / 10;
    }
    aveError = 0;
    it = 0;
    maxIt = 100;
    sumError = 0;
    twiddleInProgress = true;
    vector_it = -1;
}

PID::~PID() {}

void PID::Init(double Kp_in, double Ki_in, double Kd_in)
{
    p[0] = Kp_in;
    p[1] = Ki_in;
    p[2] = Kd_in;
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
        double aveCte = (p_error + cte) / 2;
        d_error = (d_error + aveCte - p_error) / 2; //smoothing the d_error
        p_error = aveCte;
        i_error += aveCte;
    }
    it++;
    sumError += std::abs(p_error);
    aveError = sumError / it;
}
double PID::TotalError(double speed)
{

    //double error = -p[0] * p_error - p[1] * d_error  - p[2] * i_error;
    double speedfactor = 0;

    speedfactor = (speed + 1) / 10;
    double error = -p[0] * p_error / speedfactor - p[1] * d_error / speedfactor - p[2] * i_error / speedfactor;
    std::cout << "it= " << it << ", speed = " << speed << "aveError= " << aveError << ", p_error =" << p_error << " , d_error =" << d_error << ", i_error= " << i_error << ", steering = " << error << endl;
    //eturn (-Kp * p_error - Kd * d_error - Ki * i_error);
    return error;
}

void PID::resetTwiddle()
{
    aveError = 0;
}

void PID::resetTwiddle()
{
    aveError = 0;
    it=0;
}

double PID::Twiddle(double speed)
{

    if (dp[0] < tol[0] && dp[1] < tol[1] && dp[2] < tol[2])
    {
        cout << "Twiddle Success ------------- p[0] =" << p[0] << ", p[1]" << p[1] << ", p[2]" << p[2] << endl;
        return TotalError(speed);
    }
    else
    {

        if (deepEval == true)
        {
            if (it < maxIt)
            {
                it++;
                return TotalError(speed);
            }
            else if (it == maxIt)
            {
                if (aveError < bestError)
                {
                    bestError = aveError;
                    dp[vector_it] *= 1.1;
                    resetDeepTwiddle();
                }
                else
                {
                    p[vector_it] += dp[vector_it];
                    dp[vector_it] *= 0.9;
                    resetDeepTwiddle();
                }
            }
            else
            {
            }
        }

        if (it < maxIt)
        {
            it++;
            return TotalError(speed);
        }
        else if (it == maxIt)
        {
            if
                aveError < bestError
                {
                    bestError = aveError;
                    dp[vector_it] *= 1.1;
                }
            else
            {
                p[vector_it] -= 2 * dp[vector_it];
                deepEval = true;
            }
        }
        else
        {
            vector_it++;
            if (vector_it == 3)
            {
                vector_it == 0;
            }
            p[vector_it] += dp[vector_it];

            aveError = 0;
            it = 0;
            if ()
        }
    }
}
