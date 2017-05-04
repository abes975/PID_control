#include "PID.h"
#include <numeric>
#include <cfloat>
#include <vector>


using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
    _p_error = 0;
    _i_error = 0;
    _d_error = 0;
    _prev_error = 0;
    _sum_error = 0;
    std::vector<double> _dp(3, 1.0);
    _p.push_back(_Kp);
    _p.push_back(_Ki);
    _p.push_back(_Kd);
    _isTuned = false;
}

void PID::UpdateError(double cte) {
    _p_error = _Kp * cte;
    _sum_error += cte;
    _d_error = _Kd * (cte - _prev_error);
    _prev_error = cte;
    _i_error = _Ki * _sum_error;
}

double PID::TotalError() {
  return -_p_error - _d_error - _i_error;
}

void PID::twiddle(double threshold, double cte) {
  double sum = std::accumulate(_dp.begin(), _dp.end(), 0.0);
  double err = 0;
  double best_err = DBL_MAX;
  while(sum > threshold) {
    for(int i = 0; i < _dp.size(); ++i) {
      _p[i] += _dp[i];
      UpdateError(cte);
      err = TotalError();
      if(err < best_err) {
        best_err = err;
        _dp[i] *= 1.1;
      } else {
        _p[i] -= 2 * _dp[i];
        UpdateError(cte);
        err = TotalError();
        if(err < best_err) {
          best_err = err;
          _dp[i] *= 1.1;
        } else {
          _p[i] += _dp[i];
          _dp[i] *= 0.9;
        }
      }
    }
  }
  _Kp = _p[0];
  _Kd = _p[1];
  _Ki = _p[2];
  _isTuned = true;
}

bool PID::isTuned(void) const {
  return _isTuned;
}

// Term += ki * error;
// …
// output = pTerm + iTerm + dTerm;
// if (output > maxLimit)
// iTerm -= output – maxLimit;
// output = maxLimit;
// else if (output < minLimit)
// iTerm += minLimit – output;
// output = minLimit;
// …
