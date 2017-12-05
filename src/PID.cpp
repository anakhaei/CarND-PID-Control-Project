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
    dp[1] = 2;
    dp[2] = 0.0002;
    for (unsigned int i = 0; i < 3; i++)
    {
        tol[i] = dp[i] / 10;
    }
    aveError = 0;
    it = 0;
    maxIt = 3200;
    sumError = 0;
    twiddleInProgress = true;
    vector_it = 0;
    deepEval = false;
    bestError= 1000000;
    opt_it=0;

    sum_steering=0;
    sum_steering_dot=0;
    sum_steering_dot_dot=0;
    w_steering=1.0;
    w_steering_dot=1.0;
    w_steering_dot_dot=1.0;
    previous_steering=0.0;
    previous_steering_dot=0.0;

    best_evaluation = 10000000000.0;
    evaluation=0.0;

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
    sumError += std::abs(cte);
    aveError = sumError / it;
}
double PID::TotalError(double speed)
{

    //double error = -p[0] * p_error - p[1] * d_error  - p[2] * i_error;
    double speedfactor = 0;

    speedfactor = (speed + 1) / 10;
    double steering = -p[0] * p_error / speedfactor - p[1] * d_error / speedfactor - p[2] * i_error / speedfactor;
    sum_steering +=std::abs(steering);
    double steering_dot= std::abs (steering-previous_steering);
    previous_steering = steering;
    sum_steering_dot += steering_dot;
    double steering_dot_dot = std::abs (steering_dot-previous_steering_dot);
    previous_steering_dot = steering_dot;
    sum_steering_dot_dot += steering_dot_dot;
    value = value + w_steering * std::abs(steering) + w_steering_dot * steering_dot + w_steering_dot_dot * steering_dot_dot;
    evaluation = value/it;

    std::cout << "it=" << it <<  ", vec_it=" << vector_it << ",o_i="<<opt_it<<", s=" << speed << "ev=" << evaluation << ",best_ev="<< best_evaluation<< ", p_e=" << p_error << " , d_e=" << d_error << ", i_e=" << i_error << ", st= " << steering << " p0=" << p[0] << ", p1=" << p[1] << ", p2=" << p[2] << ", dp0=" << dp[0] << ", dp1=" << dp[1] << ", dp2=" << dp[2] <<", ss="<<sum_steering<< ",ssd="<< sum_steering_dot << "ssdd="<<sum_steering_dot_dot<< endl;
    //eturn (-Kp * p_error - Kd * d_error - Ki * i_error);
    return steering;
}

void PID::resetTwiddle()
{
    aveError = 0;
    d_error = 0;
    p_error = 0;
    i_error = 0;
    isInitialized = false;
    it = 0;
    sumError = 0;
    sum_steering=0;
    sum_steering_dot=0;
    sum_steering_dot_dot=0;
    evaluation=0;
    value = 0;
    opt_it++;
}

void PID::iterateInPvector()
{
    vector_it++;
    if (vector_it == 3)
    {
        vector_it = 0;
    }
    p[vector_it] += dp[vector_it];
}

double PID::Twiddle(double speed)
{
    double steering = TotalError(speed);

    if (dp[0] < tol[0] && dp[1] < tol[1] && dp[2] < tol[2])
    {
        cout << "Twiddle Success ------------- p[0] =" << p[0] << ", p[1]" << p[1] << ", p[2]" << p[2] << ", dp[0] =" << dp[0] << ", dp[1]" << dp[1] << ", dp[2]" << dp[2] << ", d="<<deepEval<<  endl;
        return TotalError(speed);
    }
    else
    {

        if (deepEval == true)
        {
            if (it < maxIt)
            {
                it++;
                return steering;
            }
            else
            {
                if (evaluation < best_evaluation)
                {
                    best_evaluation = evaluation;
                    dp[vector_it] *= 1.1;
                    iterateInPvector();
                    cout << "Reset Twiddle from A ---------------------------------------------------------------------------------------------------------" << endl;
                    resetTwiddle();
                    deepEval = false;
                    return steering;

                }
                else
                {
                    p[vector_it] += dp[vector_it];
                    dp[vector_it] *= 0.9;
                    iterateInPvector();
                    cout << "Reset Twiddle from B ---------------------------------------------------------------------------------------------------------" << endl;
                    resetTwiddle();
                    deepEval = false;
                    return steering;
                }
            }
        }
        else
        {
            if (it < maxIt)
            {
                it++;
                return steering;
            }
            else
            {
                if (evaluation < best_evaluation)
                {

                    best_evaluation = evaluation;
                    dp[vector_it] *= 1.1;
                    iterateInPvector();
                    cout << "Reset Twiddle from C ---------------------------------------------------------------------------------------------------------" << endl;
                    resetTwiddle();
                    return steering;
                }
                else
                {
                    p[vector_it] = p[vector_it]-(2 * dp[vector_it]);
                    deepEval = true;
                    cout << "Reset Twiddle from D ---------------------------------------------------------------------------------------------------------" << endl;
                    resetTwiddle();
                    return steering;
                }
            }
        }
    }
}
