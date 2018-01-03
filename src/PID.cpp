#include "PID.h"
#include <vector>
#include <limits>
#include <numeric>
#include <cmath>
#include <ctime>
#include <iostream>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, bool twiddle_enable, double twiddle_tol, int twiddle_n, double twiddle_multiplier) {
  // twiddle init
  twiddle_dp = {0.1, 0.01, 0.01};
  this->twiddle_enable = twiddle_enable;
  this->twiddle_tol = twiddle_tol;
  this->twiddle_n = twiddle_n;
  this->twiddle_multiplier = twiddle_multiplier;
  twiddle_n_index = 0;
  twiddle_param_index = 0;
  twiddle_best_err = numeric_limits<double>::max();
  twiddle_state = ADD_PD;

  // PID init
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  prev_ticks = clock();

}

void PID::UpdateError(double cte) {
  double time_delta = (double) (clock() - prev_ticks) / CLOCKS_PER_SEC;  // in seconds
  cout << "dt: " << time_delta
       << "    t_prev: " << prev_ticks
       << "    twiddle_n: " << twiddle_n_index
       << "    twiddle_param: " << twiddle_param_index
       << "    twiddle_state: " << twiddle_state
       << "    twiddle_sum: " << accumulate(twiddle_dp.begin(), twiddle_dp.end(), 0.0) << endl;

  if (twiddle_enable) {
    vector<double *> twiddle_p = {&Kp, &Ki, &Kd};
    if (accumulate(twiddle_dp.begin(), twiddle_dp.end(), 0.0) > twiddle_tol && twiddle_n_index == 0) {
      switch(twiddle_state) {
        // Add dp to p
        case ADD_PD:
          *twiddle_p[twiddle_param_index] += twiddle_dp[twiddle_param_index];
          twiddle_state = FIRST_PASS;
          break;

        // If error improved, increase dp and end, else subtract dp from original p
        case FIRST_PASS:
          if (fabs(cte) < fabs(twiddle_best_err)) {
            twiddle_best_err = cte;
            twiddle_dp[twiddle_param_index] *= (1 + twiddle_multiplier);
            twiddle_state = DONE;
          } else {
            *twiddle_p[twiddle_param_index] -= 2 * twiddle_dp[twiddle_param_index];
            twiddle_state = SECOND_PASS;
          }
          break;

        // If error improved, increase dp, else decrease dp. End either way
        case SECOND_PASS:
          if (fabs(cte) < fabs(twiddle_best_err)) {
            twiddle_best_err = cte;
            twiddle_dp[twiddle_param_index] *= (1 + twiddle_multiplier);
          } else {
            *twiddle_p[twiddle_param_index] += twiddle_dp[twiddle_param_index];
            twiddle_dp[twiddle_param_index] *= (1 - twiddle_multiplier);
          }

        // Move to the next dp and start over
        case DONE:
          ++twiddle_param_index %= twiddle_dp.size();
          twiddle_state = ADD_PD;
          break;
      }
    }
    ++twiddle_n_index %= twiddle_n;
  }

  if (time_delta == 0) {  // Prevent divide by 0 error
    time_delta = 1;
  }
  d_error = (cte - p_error) / time_delta;  // time_delta cannot be 0
  p_error = cte;
  i_error += cte;
  prev_ticks = clock();
}

double PID::TotalError() {
  printf("K[%f, %f, %f]    err[%f, %f, %f]\n", Kp, Ki, Kd, p_error, i_error, d_error);
  return (double) -Kp * p_error - Ki * i_error - Kd * d_error;
}

