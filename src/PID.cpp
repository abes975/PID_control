#include "PID.h"
#include <cfloat>
#include <vector>
#include <iostream>


using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd)
{
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
    _p_error = 0;
    _i_error = 0;
    _d_error = 0;
    _prev_error = 0;
    _isTuned = false;
}

void PID::UpdateError(double cte)
{
    _p_error = cte;
    _d_error = (cte - _prev_error);
    _prev_error = cte;
    _i_error += cte;
    // Anti-windup
    // double term_max = 1.0;
    // double term_min = -1.0;
    // if ((_Ki * _i_error) > term_max) {
    //   _i_error = 0;
    //   std::cout << "Antiwindup ..clamped i_error to " << _i_error << std::endl;
    // }
    // else if ((_Ki * _i_error) < term_min) {
    //   _i_error = 0;
    //   std::cout << "Antiwindup ..clamped i_error to " << _i_error << std::endl;
    // }

}

double PID::TotalError()
{
  double error = -_Kp * _p_error + -_Ki * _i_error + -_Kd * _d_error;
  return error;

}

bool PID::isTuned(void) const
{
  return _isTuned;
}

void PID::setTuned(bool tuned)
{
  _isTuned = tuned;
}

void PID::setNewCoefficients(const std::vector<double>& coeff)
{
  if (coeff.size() != 3)
    return;
  _Kp = coeff.at(0);
  _Ki = coeff.at(1);
  _Kd = coeff.at(2);
}

// void PID::printErrors() {
//   std::cout << " p_error = " << _p_error << " d_error = " << _d_error << " i_error = " << _i_error << std::endl;
// }
//
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
