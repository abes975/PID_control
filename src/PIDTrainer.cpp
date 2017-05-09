#include "PIDTrainer.hpp"
#include "PID.h"
#include <numeric>
#include <iostream>

PIDTrainer::PIDTrainer(PID& p, double threshold, double resetTreshold)
{
  _pid = p;
  _threshold = threshold;
  _currState = PIDTrainer::state::INIT;
  _samples = 1;
  _best_param = std::vector<double>(3,0.0);
  _dp = std::vector<double>(3, 1.0);
  increment_step = 1.1;
  decrement_step = 0.7;
}


PIDTrainer::state PIDTrainer::getState() const
{
  return _currState;
}

const std::vector<double>& PIDTrainer::dumpCoefficient() const
{
  return _p;
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
  double error = CurrentError();
  double init_sum = 0.0;
  double sum = std::accumulate(_dp.begin(), _dp.end(), init_sum);
  // check if finished
  if(sum < _threshold) {
    std::cout << "Merdissima TRAINING COMPLETE sum = " << sum  << _threshold << std::endl;
    _currState = TRAINING_COMPLETE;
  }
  if (_currState == INIT)
    _best_error = error;

  switch(_currState) {
    case state::INIT:
        std::cout << "INIT" << std::endl;
        //_dp = std::vector<double>(3,1.0);
        _p = std::vector<double>(3, 0.0);
        //_pid.Init(_p[0], _p[1], _p[2]);
        _next_index = 0;
        //std::cout << "INIT " << " i parametri " << _p[0] << " " << _p[1] << " " << _p[2] << std::endl;
        _currState = INCREASE_COEFFICIENT;
      break;
    case INCREASE_COEFFICIENT:
      std::cout << "INCREASE COEFFICIENT next index = " << _next_index << std::endl;
      _p[_next_index] += _dp[_next_index];
      std::cout << " sto per inizializzare " << std::endl;
      _pid.Init(_p[0], _p[1], _p[2]);
      std::cout << " inizialiizato " << std::endl;
      _currState = EVALUATE_ERROR_AFTER_INCREASE;
      std::cout << "finito stato automa " << std::endl;
      break;
    case EVALUATE_ERROR_AFTER_INCREASE:
      std::cout << "EVALUATE ERROR AFTER INCREASE next index = " << _next_index << std::endl;
      if (error < _best_error) {
        _best_error = error;
        _next_index = (_next_index + 1) % 3;
        _currState = INCREASE_COEFFICIENT;
      } else {
        _p[_next_index] -= 2 * _dp[_next_index];
        _pid.Init(_p[0], _p[1], _p[2]);
        _currState = EVALUATE_ERROR_AFTER_DECREASE;
      }
      break;
    case EVALUATE_ERROR_AFTER_DECREASE:
      if(error < _best_error) {
        _best_error = error;
        _dp[_next_index] *= increment_step;
        // not sure about this
        _best_param = _p;
      } else {
				_p[_next_index] += _dp[_next_index];
				_dp[_next_index] *= decrement_step;
			}
      _next_index = (_next_index + 1) % 3;
      _currState = INCREASE_COEFFICIENT;
      break;
    case TRAINING_COMPLETE:
      std::cout << "Training iteration complete" << std::endl;
      return;
    default:
      std::cout << "unknown state merdissima " << std::endl;
  }
  _total_error = 0;
  _samples = 1;
}
