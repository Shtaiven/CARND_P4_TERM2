#ifndef PID_H
#define PID_H

#include <vector>

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
  double twiddle_total_err;
  double twiddle_best_err;
  double twiddle_tol;
  std::vector<double> twiddle_dp;
  unsigned long int twiddle_cnt;
  bool twiddle_done;
  bool twiddle_second_pass;
  int twiddle_param_index;
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
  void Init(double Kp, double Ki, double Kd, double twiddle_tol=0.2);

  /*
  * Twiddle-dee! Twiddle-dum!
  */
  void Twiddle(unsigned long int n);

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
