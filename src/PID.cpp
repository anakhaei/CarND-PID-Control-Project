#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
    d_error=0;
    p_error=0;
    i_error=0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  Kp=Kp;
  Ki=ki;
  Kd=kd;
}

void PID::UpdateError(double cte) {
    d_error = p_error-cte;
    p_error=cte;
    i_error +=cte;

}

double PID::TotalError() {
    return -kp * p_error - kd * d_error - ki * i_error;
}

