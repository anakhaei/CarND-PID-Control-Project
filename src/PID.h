#ifndef PID_H
#define PID_H

#include <vector>
#include <cmath> 

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  //double Kp;
  //double Ki;
  //double Kd;
  std::vector<double> p;

  bool isInitialized;

  /*
  * Twiddle parameters
  */ 
  bool twiddleActive;
  std::vector<double> dp;
  std::vector<double> tol;
  double aveError;
  double it;
  double maxIt;
  double sumError;
  bool twiddleInProgress;
  int vector_it;
  bool deepEval;
  double bestError;
  int opt_it;
  double sum_steering;
  double sum_steering_dot;
  double sum_steering_dot_dot;
  double evaluation;
  double best_evaluation;
    double value;
  double w_steering;
  double w_steering_dot;
  double w_steering_dot_dot;
  double previous_steering;
  double previous_steering_dot;

 





  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp_in, double Ki_in, double Kd_in);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError(double speed);

  /*
  * Fine tune parameters.
  */
  double Twiddle(double speed);

  void resetTwiddle();

  void iterateInPvector();
};

#endif /* PID_H */
