#ifndef PID_H
#define PID_H

#include <vector>
#include <ctime>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Twiddle-dee! Twiddle-dum!
  */
  bool twiddle_enable;
  double twiddle_best_err;
  double twiddle_tol;
  double twiddle_multiplier;
  std::vector<double> twiddle_dp;
  enum twiddle_states {ADD_PD, FIRST_PASS, SECOND_PASS, DONE};
  int twiddle_state;
  int twiddle_param_index;
  int twiddle_n;
  int twiddle_n_index;
  clock_t prev_ticks;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

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
  void Init(double Kp, double Ki, double Kd, bool twiddle_enable=false, double twiddle_tol=0.2, int twiddle_n = 100, double twiddle_multiplier=0.1);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
