#include "PID.h"
#include <vector>
#include <limits>
#include <numeric>
#include <cmath>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double twiddle_tol) {
  twiddle_dp = {1.0, 1.0, 1.0};
  this->twiddle_tol = twiddle_tol;
  twiddle_done = false;
  twiddle_cnt = 0;
  twiddle_second_pass = false;
  twiddle_param_index = 0;
  twiddle_total_err = 0;
  twiddle_best_err = numeric_limits<double>::max();
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}

void PID::Twiddle(unsigned long int n) {
  // n is the number of iterations that must be passed before the coefficients are modified
  vector<double *> coefficients = {&Kp, &Ki, &Kd};
  twiddle_total_err += pow(p_error, 2);
  if (++twiddle_cnt >= n) {
    twiddle_total_err /= n;
    if (accumulate(twiddle_dp.begin(), twiddle_dp.end(), 0) > twiddle_tol) {
      if (!twiddle_second_pass) {
        *coefficients[twiddle_param_index] += twiddle_dp[twiddle_param_index];
        if (twiddle_total_err < twiddle_best_err) {
          twiddle_best_err = twiddle_total_err;
          twiddle_dp[twiddle_param_index] *= 1.1;
          twiddle_param_index++;
          twiddle_total_err = 0;
        } else {
          twiddle_second_pass = true;
        }
      } else {
        *coefficients[twiddle_param_index] -= twiddle_dp[twiddle_param_index] * 2;
        if (twiddle_total_err < twiddle_best_err) {
          twiddle_best_err = twiddle_total_err;
          twiddle_dp[twiddle_param_index] *= 1.1;
        } else {
          *coefficients[twiddle_param_index] += twiddle_dp[twiddle_param_index];
          twiddle_dp[twiddle_param_index] *= 0.9;
        }
        twiddle_second_pass = false;
        twiddle_param_index++;
        twiddle_total_err = 0;
      }
    } else {
      twiddle_done = true;
    }
  }
  twiddle_param_index %= coefficients.size();
  twiddle_cnt %= n;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() {
  return (double) -Kp * p_error - Ki * i_error - Kd * d_error;
}

