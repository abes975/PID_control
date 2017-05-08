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
      twiddle(cte);
    //}
  }
}

#if 0
void PIDTrainer::twiddle(double cte)
{
  switch(_currState) {
    case state::INIT:
        //double init_sum = 0;
        _dp = std::vector<double>(3,1.0);
        _p = std::vector<double>(3,0.0);
        _pid.Init(_p[0], _p[1], _p[2]);
        std::cout << "INIT " << " i parametri " << _p[0] << " " << _p[1] << " " << _p[2] << std::endl;
        _processed_samples = 0;
        _currState = state::COMPUTE_BEST_ERROR;
    break;
    case state::COMPUTE_BEST_ERROR:
      _pid.UpdateError(cte);
      _processed_samples++;
      if(_processed_samples > 50) {
        _best_error = _pid.TotalError();
        // do I need to reset here?
        _processed_samples = 0;
        _currState = state::CHECK_FINISHED;
      } else
        _currState = state::COMPUTE_BEST_ERROR;
    break;
    case state::CHECK_FINISHED:
      double sum = std::accumulate(dp.begin(), dp.end(), 0.0);
      if (sum > _threshold)
        _currState = state::TRAINING_COMPLETE;
      else
        _currState = state::EVALUATE_ERROR_P1;
        // Reset simulator
    break;
    case state::EVALUATE_ERROR_P1:
      _p[0] += dp[0];
      std::cout << "\t  _p[" << i << "] = " << _p[i] << " dp[" << i << "] = " << dp[i] << std::endl;
      _pid.Init(_p[0], _p[1], _p[2]);
      _pid.UpdateErrors(cte);
      _samples++;
      _currState = state::NEED_MORE_SAMPLES_P1;
    break;
    case state::NEED_MORE_SAMPLES_P1:
      _pid.UpdateErrors(cte);
      _samples++;
      if (_samples < 50) {
        _currState = state::NEED_MORE_SAMPLES_P1;
      } else {
        _samples = 0;
        _err = pid.TotalError();
        if(_err < _best_err) {
          _best_err = _err;
          _dp[0] *= 1.1;
        } else {
          _p[0] -= 2 * dp[0];
          _pid.Init(_p[0], _p[1], _p[2]);
          // RESET simulator
          _currState = NEED_MORE_SAMPLES_2_P1;
      }
    break;
    case state::NEED_MORE_SAMPLES_2_P1:
      _pid.UpdateErrors(cte);
      _samples++;
      if (_samples < 50) {
        _pid.UpdateErrors(cte);
        _currState = state::NEED_MORE_SAMPLES_2_P1;
      } else {
        _samples = 0;
        _err = pid.TotalError();
        if(_err < _best_err) {
          _best_err = _err;
          _dp[0] *= 1.1;
        } else {
          _p[0] += _dp[0];
          _dp[0] *= 0.9;
          if(_next_index < 2) {
            _next_index = _nextindex++;
            _curr_state = _EVALUATE_ERROR_P1;
            // reset
          } else {
            _currState = CHECK_FINISHED;
          }
      }
    break;
    default:
      std::cout << "BUUUGGGG" << std::endl;
  }
}
#endif

//
//   double sum = std::accumulate(dp.begin(), dp.end(), init_sum);
//   // Kp, Ki, Kd
//   PID pid;
//   pid.Init(_p[0], _p[1], _p[2]);
//
//   UpdateErrors(pid);
//   double best_err = pid.TotalError();
//   double err;
//   int iteration = 1;
//   while(sum > _threshold) {
//     std::cout << "Twiddle : iteration: " << iteration << " best error = " << best_err << std::endl;
//     for(int i = 0; i < dp.size(); ++i) {
//       _p[i] += dp[i];
//       std::cout << "\t  _p[" << i << "] = " << _p[i] << " dp[" << i << "] = " << dp[i] << std::endl;
//       PID pid;
//       pid.Init(_p[0], _p[1], _p[2]);
//       UpdateErrors(pid);
//       err = pid.TotalError();
//       if(err < best_err) {
//         std::cout << "\terror < best error " << err << " " << best_err;
//         best_err = err;
//         dp[i] *= 1.1;
//         std::cout << " new dp[" << i << "] = " << dp[i] << std::endl;
//       } else {
//         _p[i] -= 2 * dp[i];
//         std::cout << "\t error >= best error " << err << " " << best_err << " new p[" << i << "] = " << _p[i] << std::endl;
//         PID pid;
//         pid.Init(_p[0], _p[1], _p[2]);
//         UpdateErrors(pid);
//         err = pid.TotalError();
//         if(err < best_err) {
//           std::cout << "\t2nd branch error < best error " << err << " " << best_err;
//           best_err = err;
//           dp[i] *= 1.1;
//           std::cout << " new dp[" << i << "] = " << dp[i] << std::endl;
//         } else {
//           _p[i] += dp[i];
//           dp[i] *= 0.9;
//           std::cout << "\t2n branch TWIDDLE  _p[" << i << "] = " << _p[i] << " dp[" << i << "] = " << dp[i] << std::endl;
//         }
//       }
//     }
//     sum = std::accumulate(dp.begin(), dp.end(), init_sum);
//     iteration++;
//   }
//   //std::cout << " LA somma e " << sum << " i parametri " << _p[0] << " " << _p[1] << " " << _p[2] << std::endl;
//   std::cout << "XXXXXXXXXXXX finito il tweedle " << " kp " << _p[0] << " ki = " << _p[1] << " kd = " << _p[2] << std::endl;
// }


void PIDTrainer::twiddle(double cte)
{
  double init_sum = 0;
  // if(_firstRun) {
  //   std::vector<double> dp = std::vector<double>(3,1.0);
  //   _p = std::vector<double>(3,0.0);
  // }
  double sum = std::accumulate(_dp.begin(), _dp.end(), init_sum);
  _dp = std::vector<double>(3,1.0);
  // Kp, Ki, Kd
  //PID pid;
  //pid.Init(_p[0], _p[1], _p[2]);
  _pid.setNewCoefficients(_p);
  //UpdateErrors(pid);
  _pid.UpdateError(cte);
  double best_err = _pid.TotalError();
  std::cout << "Prima del loop tweedle LA somma e " << sum << " i parametri p = " << _p[0] << " " << _p[1] << " " << _p[2] <<
    " d = " << _dp[0] << " " << _dp[1] << " " << _dp[2] << " best_err = " << best_err << std::endl;
  double err;
  int iteration = 1;
  while(sum > _threshold) {
    //std::cout << "Twiddle : iteration: " << iteration << " best error = " << best_err << std::endl;
    for(int i = 0; i < _dp.size(); ++i) {
      _p[i] += _dp[i];
      //std::cout << "\t  _p[" << i << "] = " << _p[i] << " dp[" << i << "] = " << _dp[i] << std::endl;
      // PID pid;
      // pid.Init(_p[0], _p[1], _p[2]);
      //UpdateErrors(pid);
      _pid.setNewCoefficients(_p);
      err = _pid.TotalError();
      std::cout << " Error = " << err << std::endl;
      if(err < best_err) {
        //std::cout << "\terror < best error " << err << " " << best_err;
        best_err = err;
        _dp[i] *= 1.1;
        //std::cout << " new dp[" << i << "] = " << _dp[i] << std::endl;
      } else {
        _p[i] -= 2 * _dp[i];
        //std::cout << "\t error >= best error " << err << " " << best_err << " new p[" << i << "] = " << _p[i] << std::endl;
        // PID pid;
        // pid.Init(_p[0], _p[1], _p[2]);
        //UpdateErrors(pid);
        _pid.setNewCoefficients(_p);
        err = _pid.TotalError();
        std::cout << " Error = " << err << std::endl;
        if(err < best_err) {
          //std::cout << "\t2nd branch error < best error " << err << " " << best_err;
          best_err = err;
          _dp[i] *= 1.1;
          //std::cout << " new dp[" << i << "] = " << _dp[i] << std::endl;
        } else {
          _p[i] += _dp[i];
          _dp[i] *= 0.9;
          //std::cout << "\t2n branch TWIDDLE  _p[" << i << "] = " << _p[i] << " dp[" << i << "] = " << _dp[i] << std::endl;
        }
      }
    }
    sum = std::accumulate(_dp.begin(), _dp.end(), init_sum);
    std::cout << "\tTWIDDLE iteration finished  _p = " << _p[0] << " " << _p[1] << " " << _p[2] << " _d = " << _dp[0] << " " << _dp[1] << " " << _dp[2] << std::endl;
    iteration++;
  }
  //std::cout << " LA somma e " << sum << " i parametri " << _p[0] << " " << _p[1] << " " << _p[2] << std::endl;
  std::cout << "XXXXXXXXXXXX finito il tweedle " << " kp " << _p[0] << " ki = " << _p[1] << " kd = " << _p[2] << std::endl;
}
