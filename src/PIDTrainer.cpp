#include "PIDTrainer.hpp"
#include "PID.h"
#include <numeric>
#include <iostream>

PIDTrainer::PIDTrainer(PID& p, double threshold, double resetTreshold)
{
  _p.push_back(p.getKp());
  _p.push_back(p.getKi());
  _p.push_back(p.getKd());
  _threshold = threshold;
  _resetTreshold = resetTreshold;
  _needReset = false;
  _currState = PIDTrainer::state::INIT;
  _pid.Init(0.0,0.0,0.0);
}

void PIDTrainer::incError(double error)
{
  _proc_samples++;
  _totalError += error * error;
}


PIDTrainer::state PIDTrainer::getState() const
{
  return _currState;
}

void PIDTrainer::saveSamples(double sample)
{
  _samples.push_back(sample);
}

bool PIDTrainer::needReset() const
{
  return _needReset;
}

const std::vector<double>& PIDTrainer::dumpCoefficient() const
{
  return _p;
}


void PIDTrainer::UpdateErrors(PID &pid)
{
  std::vector<double>::const_iterator cit;
  for(cit = _samples.begin(); cit != _samples.end(); ++cit) {
      pid.UpdateError(*cit);
  }
}

void PIDTrainer::TuneParameters(double cte)
{
   if(abs(cte) > _resetTreshold) {
    _needReset = true;
    return;
   } else {
    _needReset = false;
    saveSamples(cte);
    //if(_samples.size() > 1) {
    //std::cout << " Ho " << _samples.size() << " samples now twiddle " << std::endl;
      twiddle(cte, 10);
    //}
  }
}

void PIDTrainer::twiddle(double cte, int samples=10)
{
  double sample_limit = samples;
  double init_sum;
  double sum;
  _needReset = false;
  switch(_currState) {
    case state::INIT:
        std::cout << "INIT" << std::endl;
        _dp = std::vector<double>(3,1.0);
        _p = std::vector<double>(3,0.0);
        _pid.Init(_p[0], _p[1], _p[2]);
        //std::cout << "INIT " << " i parametri " << _p[0] << " " << _p[1] << " " << _p[2] << std::endl;
        _proc_samples = 0;
        _currState = state::COMPUTE_BEST_ERROR;
      break;
    case state::COMPUTE_BEST_ERROR:
      //std::cout << "COMPUTE BEST ERROR" << std::endl;
      _pid.UpdateError(cte);
      _proc_samples++;
      if(_proc_samples >= sample_limit) {
        _best_error = _pid.TotalError();
        // do I need to reset here?
        _proc_samples = 0;
        _currState = state::CHECK_FINISHED;
      } else {
        _currState = state::COMPUTE_BEST_ERROR;
      }
      break;
    case state::CHECK_FINISHED:
      std::cout << "CHECK_FINISHED" << std::endl;
      init_sum = 0.0;
      sum = std::accumulate(_dp.begin(), _dp.end(), init_sum);
      std::cout << "XXXXXXXXXXX SUM = " << sum << " _treshold = " << _threshold << std::endl;
      if (sum <= _threshold) {
        _currState = state::TRAINING_COMPLETE;
      } else {
        _next_index = 0;
        _currState = state::EVALUATE_ERROR;
        _needReset = true;
      }
      break;
    case state::EVALUATE_ERROR:
      //std::cout << "EVALUATE ERROR for param index " << _next_index << std::endl;
      _p[_next_index] += _dp[_next_index];
      //std::cout << "\t  _p[" << _next_index << "] = " << _p[_next_index] << " dp[" << _next_index << "] = " << _dp[_next_index] << std::endl;
      _pid.Init(_p[0], _p[1], _p[2]);
      _pid.UpdateError(cte);
      _proc_samples++;
      _currState = state::NEED_MORE_SAMPLES;
      break;
    case state::NEED_MORE_SAMPLES:
      _pid.UpdateError(cte);
      _proc_samples++;
      //std::cout << "NEED_MORE_SAMPLES processed = " << _proc_samples << " for param index " << _next_index << std::endl;
      if (_proc_samples < sample_limit) {
        _currState = state::NEED_MORE_SAMPLES;
      } else {
        _proc_samples = 0;
        _error = _pid.TotalError();
        if(_error < _best_error) {
          _best_error = _error;
          _dp[_next_index] *= 1.1;
          if(_next_index == 2) {
            _currState = CHECK_FINISHED;
          } else {
            _next_index++;
            _currState = EVALUATE_ERROR;
            _needReset = true;
          }
        } else {
          _p[_next_index] -= 2 * _dp[_next_index];
          _pid.Init(_p[0], _p[1], _p[2]);
          _currState = NEED_MORE_SAMPLES_2;
        }
        _needReset = true;
      }
      break;
    case state::NEED_MORE_SAMPLES_2:
      //std::cout << "NEED_MORE_SAMPLES 2 processed = " << _proc_samples << " for param index " << _next_index << std::endl;
      _pid.UpdateError(cte);
      _proc_samples++;
      if (_proc_samples < 1) {
        _pid.UpdateError(cte);
        _currState = state::NEED_MORE_SAMPLES_2;
      } else {
        _proc_samples = 0;
        _error = _pid.TotalError();
        if(_error < _best_error) {
          _best_error = _error;
          _dp[_next_index] *= 1.1;
          if(_next_index == 2) {
            _currState = CHECK_FINISHED;
          } else {
            _next_index++;
            _currState = EVALUATE_ERROR;
            _needReset = true;
          }
        } else {
          _p[_next_index] += _dp[_next_index];
          _dp[_next_index] *= 0.9;
          _next_index = (_next_index+1)% 3;
          if(_next_index == 2) {
            _currState = CHECK_FINISHED;
          } else {
            _next_index++;
            _currState = EVALUATE_ERROR;
            _needReset = true;
          }
        }
      }
      break;
    case state::TRAINING_COMPLETE:
      //_isTuned = true;
      _currState = INIT;
    break;
    default:
      std::cout << "BUUUGGGG" << std::endl;
  }
}



// void PIDTrainer::twiddle(double cte)
// {
//   double init_sum = 0;
//   // if(_firstRun) {
//   //   std::vector<double> dp = std::vector<double>(3,1.0);
//   //   _p = std::vector<double>(3,0.0);
//   // }
//   double sum = std::accumulate(_dp.begin(), _dp.end(), init_sum);
//   _dp = std::vector<double>(3,1.0);
//   // Kp, Ki, Kd
//   //PID pid;
//   //pid.Init(_p[0], _p[1], _p[2]);
//   _pid.setNewCoefficients(_p);
//   //UpdateErrors(pid);
//   _pid.UpdateError(cte);
//   double best_err = _pid.TotalError();
//   std::cout << "Prima del loop tweedle LA somma e " << sum << " i parametri p = " << _p[0] << " " << _p[1] << " " << _p[2] <<
//     " d = " << _dp[0] << " " << _dp[1] << " " << _dp[2] << " best_err = " << best_err << std::endl;
//   double err;
//   int iteration = 1;
//   while(sum > _threshold) {
//     //std::cout << "Twiddle : iteration: " << iteration << " best error = " << best_err << std::endl;
//     for(int i = 0; i < _dp.size(); ++i) {
//       _p[i] += _dp[i];
//       //std::cout << "\t  _p[" << i << "] = " << _p[i] << " dp[" << i << "] = " << _dp[i] << std::endl;
//       // PID pid;
//       // pid.Init(_p[0], _p[1], _p[2]);
//       //UpdateErrors(pid);
//       _pid.setNewCoefficients(_p);
//       err = _pid.TotalError();
//       std::cout << " Error = " << err << std::endl;
//       if(err < best_err) {
//         //std::cout << "\terror < best error " << err << " " << best_err;
//         best_err = err;
//         _dp[i] *= 1.1;
//         //std::cout << " new dp[" << i << "] = " << _dp[i] << std::endl;
//       } else {
//         _p[i] -= 2 * _dp[i];
//         //std::cout << "\t error >= best error " << err << " " << best_err << " new p[" << i << "] = " << _p[i] << std::endl;
//         // PID pid;
//         // pid.Init(_p[0], _p[1], _p[2]);
//         //UpdateErrors(pid);
//         _pid.setNewCoefficients(_p);
//         err = _pid.TotalError();
//         std::cout << " Error = " << err << std::endl;
//         if(err < best_err) {
//           //std::cout << "\t2nd branch error < best error " << err << " " << best_err;
//           best_err = err;
//           _dp[i] *= 1.1;
//           //std::cout << " new dp[" << i << "] = " << _dp[i] << std::endl;
//         } else {
//           _p[i] += _dp[i];
//           _dp[i] *= 0.9;
//           //std::cout << "\t2n branch TWIDDLE  _p[" << i << "] = " << _p[i] << " dp[" << i << "] = " << _dp[i] << std::endl;
//         }
//       }
//     }
//     sum = std::accumulate(_dp.begin(), _dp.end(), init_sum);
//     std::cout << "\tTWIDDLE iteration finished  _p = " << _p[0] << " " << _p[1] << " " << _p[2] << " _d = " << _dp[0] << " " << _dp[1] << " " << _dp[2] << std::endl;
//     iteration++;
//   }
//   //std::cout << " LA somma e " << sum << " i parametri " << _p[0] << " " << _p[1] << " " << _p[2] << std::endl;
//   std::cout << "XXXXXXXXXXXX finito il tweedle " << " kp " << _p[0] << " ki = " << _p[1] << " kd = " << _p[2] << std::endl;
// }
