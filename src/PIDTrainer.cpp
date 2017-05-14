#include "PIDTrainer.hpp"
#include "PID.h"
#include <numeric>
#include <limits>
#include <iostream>
#include <cfloat>

PIDTrainer::PIDTrainer(PID* p, double threshold)
{
  _pid = p;
  _threshold = threshold;
  _currState = PIDTrainer::state::TRAINING_INIT;
  _samples = 1;
  _param = std::vector<double>(3,0.0);
  _best_param = std::vector<double>(3,0.0);
  _dp = std::vector<double>(3, 0.2);
  increment_step = std::vector<double>(3,0.0);
  increment_step[0] = 1.3;
  increment_step[1] = 1.3;
  increment_step[2] = 1.3;
  decrement_step = std::vector<double>(3,0.0);
  decrement_step[0] = 0.5;
  decrement_step[1] = 0.5;
  decrement_step[2] = 0.5;
  _best_error = -1;
  _total_train = 1;
  _total_error = FLT_MAX;
  _best_error = FLT_MAX;
}


const std::vector<double>& PIDTrainer::dumpCoefficient() const
{
  return _param;
}


void PIDTrainer::UpdateError(double cte)
{
  _samples++;
  _total_error += cte * cte;
}

double PIDTrainer::CurrentError()
{
  return _total_error / _samples;
}


void PIDTrainer::TuneParameters()
{
  // Get current error (I have to do always)
  double error = CurrentError();
  double init_sum = 0.0;

  double sum = std::accumulate(_dp.begin(), _dp.end(), init_sum);
  // check if finished
  if(_currState != TRAINING_INIT && sum < _threshold) {
    std::cout << "TRAINING COMPLETE " <<" Best Param = " <<
      " Kp = " << _param[0] << " Kd = " << _param[1] <<
      " Ki = " << _param[2] << std::endl;
    _best_param = _param;
    _currState = TRAINING_COMPLETE;
  }

  switch(_currState) {
    case state::TRAINING_INIT:
      _best_error = error;
      _param[0] = _pid->getKp();
      _param[1] = _pid->getKd();
      _param[2] = _pid->getKi();
      _next_index = 0;
      _param[_next_index] += _dp[_next_index];
      _pid->Init(_param[0], _param[2], _param[1]);
      _currState = EVALUATE_ERROR_AFTER_INCREASE;
    break;
    case EVALUATE_ERROR_AFTER_INCREASE:
      if (error < _best_error) {
        _best_error = error;
        _dp[_next_index] *= increment_step[_next_index];
        _next_index = (_next_index + 1) % 3;

        _param[_next_index] += _dp[_next_index];
        _pid->Init(_param[0], _param[2], _param[1]);

        _currState = EVALUATE_ERROR_AFTER_INCREASE;
      } else {
        _param[_next_index] -= 2 * _dp[_next_index];
        _pid->Init(_param[0], _param[2], _param[1]);
        _currState = EVALUATE_ERROR_AFTER_DECREASE;
      }
    break;
    case EVALUATE_ERROR_AFTER_DECREASE:
      if(error < _best_error)
        _dp[_next_index] *= increment_step[_next_index];
      else {
		    _param[_next_index] += _dp[_next_index];
		    _dp[_next_index] *= decrement_step[_next_index];
      }
      _next_index = (_next_index + 1) % 3;

      _param[_next_index] += _dp[_next_index];
      _pid->Init(_param[0], _param[2], _param[1]);

      _currState = EVALUATE_ERROR_AFTER_INCREASE;
    break;
    case TRAINING_COMPLETE:
      _total_train++;
      _best_param = _param;
      _dp = std::vector<double>(3, 0.2);
      _pid->Init(_param[0], _param[2], _param[1]);
      _pid->setTuned(true);
      _currState = TRAINING_INIT;
      break;
    default:
      std::cout << "unknown state " << std::endl;
  }
  _total_error = 0;
  _samples = 1;
}
