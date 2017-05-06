#include "PID.h"
#include <numeric>
#include <cfloat>
#include <vector>
#include <iostream>


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
    for(int i = 0 ; i < 3; i++)
      _dp.push_back(1.0);
    _p.push_back(_Kp);
    _p.push_back(_Ki);
    _p.push_back(_Kd);
    _isTuned = false;
}

void PID::UpdateError(double cte) {
    _p_error = cte;
    _i_error += cte;
    _d_error = (cte - _prev_error);
    _prev_error = cte;
    // std::cout << "Finita la update errors " << std::endl;
    // printErrors();
}

void PID::UpdateError(vector<double> ctes) {
    for(double cte : ctes) {
      _p_error = cte;
      _i_error += cte;
      _d_error = (cte - _prev_error);
      _prev_error = cte;
    }
}

std::vector<double> PID::saveErrors() {
  vector<double> backup;
  backup.push_back(_p_error);
  backup.push_back(_i_error);
  backup.push_back(_d_error);
  backup.push_back(_prev_error);
  return backup;
}

void PID::restoreErrors(std::vector<double>& err) {
  _p_error = err.at(0);
  _i_error = err.at(1);
  _d_error = err.at(2);
  _prev_error = err.at(3);
}


double PID::TotalError() {
  return -_Kp * _p_error -_Ki * _i_error -_Kd * _d_error;
}

double PID::TotalError(std::vector<double>& p) {
  return -p[0] * _p_error -p[1] * _i_error -p[2] * _d_error;
}

void PID::twiddle(double threshold, vector<double> ctes) {
  vector<double> a;
  double init_sum = 0;
  _dp[0] = 1.0;
  _dp[1] = 1.0;
  _dp[2] = 1.0;
  double sum = std::accumulate(_dp.begin(), _dp.end(), init_sum);
  a = saveErrors();
  UpdateError(ctes);
  double best_err = TotalError(_p);
  double err;
  while(sum > threshold) {
    for(int i = 0; i < _dp.size(); ++i) {
      _p[i] += _dp[i];
      cout << " TWIDDLE 1 _p[" << i << "] = " << _p[i] << " _dp[" << i << "] = " << _dp[i] << std::endl;
      restoreErrors(a);
      UpdateError(ctes);
      err = TotalError(_p);
      std::cout << " Sono nella twiddle 1 err = " << err << std::endl;
      if(err < best_err) {
        best_err = err;
        _dp[i] *= 1.1;
      } else {
        _p[i] -= 2 * _dp[i];
        cout << " TWIDDLE 2 _p[" << i << "] = " << _p[i] << " _dp[" << i << "] = " << _dp[i] << std::endl;
        restoreErrors(a);
        UpdateError(ctes);
        err = TotalError(_p);
        std::cout << " Sono nella twiddle 2 err " << err << std::endl;
        if(err < best_err) {
          best_err = err;
          _dp[i] *= 1.1;
        } else {
          _p[i] += _dp[i];
          _dp[i] *= 0.9;
        }
      }
    }
    sum = std::accumulate(_dp.begin(), _dp.end(), init_sum);
  }
  std::cout << "XXXXXXXXXXXX finito il tweedle " << " kp " << _p[0] << " ki = " << _p[1] << " kd = " << _p[2] << std::endl;
  _Kp = _p[0];
  _Ki = _p[1];
  _Kd = _p[2];
  restoreErrors(a);
  std::cout << "XXXXXXXXXXXXXXXXXXXX FINITA RESTORED ERROR " << std::endl;
  printErrors();
  _isTuned = true;
}

bool PID::isTuned(void) const {
  return _isTuned;
}

void PID::printErrors() {
  std::cout << " p_error = " << _p_error << " d_error = " << _d_error << " i_error = " << _i_error << std::endl;
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
