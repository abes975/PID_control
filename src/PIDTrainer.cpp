#include "PIDTrainer.hpp"
#include "PID.h"
#include <numeric>
#include <iostream>
#include <cfloat>

PIDTrainer::PIDTrainer(PID* p, double threshold, double resetTreshold)
{
  _pid = p;
  _threshold = threshold;
  _currState = PIDTrainer::state::TRAINING_INIT;
  _samples = 1;
  _param = std::vector<double>(3,0.0);
  _best_param = std::vector<double>(3,0.0);
  _dp = std::vector<double>(3, 0.1);
  increment_step = 1.1;
  decrement_step = 0.9;
  _best_error = -1;
  _total_train = 1;
  _total_error = FLT_MAX;
 _best_error = FLT_MAX;
}


PIDTrainer::state PIDTrainer::getState() const
{
  return _currState;
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
  std::cout << "TuneParameter Current error = " << error << std::endl;
  double init_sum = 0.0;

  // _param[0] = _pid->getKp();
  // _param[1] = _pid->getKi();
  // _param[2] = _pid->getKd();

  // if (_currState == INIT) {
  //   std::cout << " XXXXX why am I entering here?" << std::endl;
  //   _dp = std::vector<double>(3, *std::min_element(_dp.begin(), _dp.end(), abs_compare));
  // }

  std::cout << " Current parameter dp[0] = " << _dp[0] << " dp1 = " << _dp[1] << " dp2 = " << _dp[2] << std::endl;

  double sum = std::accumulate(_dp.begin(), _dp.end(), init_sum);
  // check if finished
  if(_currState != TRAINING_INIT && sum < _threshold) {
    std::cout << "TRAINING COMPLETE sum = " << sum  << " Threshold = " << _threshold <<
      " Best Param = " << " Kp = " << _param[0] << " Ki = " << _param[1] <<
      " Kd = " << _param[2] << std::endl;
    _best_param = _param;
    _currState = TRAINING_COMPLETE;
  }

  switch(_currState) {
    case state::TRAINING_INIT:
        std::cout << "TRAINING INIT" << std::endl;
        _best_error = error;
        std::cout << "\tGet the 1st TIME best error " << _best_error << std::endl;
        _dp = std::vector<double>(3, 0.2);
        //_param = std::vector<double>(3, 0.0);
        // _param[0] = _pid->getKp();
        // _param[1] = _pid->getKi();
        // _param[2] = _pid->getKd();
        _pid->Init(_param[0], _param[1], _param[2]);
        _next_index = 0;
        //std::cout << "TRAINING INIT " << " i parametri " << _param[0] << " " << _param[1] << " " << _param[2] << std::endl;
        _currState = INCREASE_COEFFICIENT;
    break;
    // case state::INIT:
    //     std::cout << "INIT" << std::endl;
    //     _dp = std::vector<double>(3,0.3);
    //     //_param = std::vector<double>(3, 0.0);
    //     //_pid.Init(_param[0], _param[1], _param[2]);
    //     // _param[0] = _pid->getKp();
    //     // _param[1] = _pid->getKi();
    //     // _param[2] = _pid->getKd();
    //     _next_index = 0;
    //     std::cout << "\tINIT " << " i parametri " << _param[0] << " " << _param[1] << " " << _param[2] << std::endl;
    //     _currState = INCREASE_COEFFICIENT;
    //   break;
    case INCREASE_COEFFICIENT:
      std::cout << "INCREASE COEFFICIENT next index = " << _next_index << std::endl;
      _param[_next_index] += _dp[_next_index];
      std::cout << "\tPID PARAMETER SHOULD BE Kp = " << _param[0] <<  " ki " << _param[1] << " kd " << _param[2] << std::endl;
      _best_param = _param;
      _pid->Init(_param[0], _param[1], _param[2]);
      _currState = EVALUATE_ERROR_AFTER_INCREASE;
      break;
    case EVALUATE_ERROR_AFTER_INCREASE:
      std::cout << "EVALUATE ERROR AFTER INCREASE on index = " << _next_index << std::endl;
      if (error < _best_error) {
        std::cout << "\tBest error decreased " << std::endl;
        _best_error = error;
        _dp[_next_index] *= increment_step;
        _best_param = _param;
        _next_index = (_next_index + 1) % 3;
        std::cout << "\tNext index will be " << _next_index << std::endl;
        _currState = INCREASE_COEFFICIENT;
      } else {
        std::cout << "\tBest error NOT decreased " << std::endl;
        _param[_next_index] -= 2 * _dp[_next_index];
        std::cout << "\tPID PARAMETER SHOULD BE Kp = " << _param[0] <<  " ki " << _param[1] << " kd " << _param[2] << std::endl;
        _best_param = _param;
        _pid->Init(_param[0], _param[1], _param[2]);
        _currState = EVALUATE_ERROR_AFTER_DECREASE;
      }
      break;
    case EVALUATE_ERROR_AFTER_DECREASE:
      std::cout << "EVALUATE ERROR AFTER DECREASE on index = " << _next_index << std::endl;
      if(error < _best_error) {
        std::cout << "\tBest error decreased " << std::endl;
        _best_error = error;
        _dp[_next_index] *= increment_step;
        // not sure about this
        _best_param = _param;
      } else {
        std::cout << "\tBest error NOT decreased " << std::endl;
		    _param[_next_index] += _dp[_next_index];
        std::cout << "\tPID PARAMETER SHOULD BE Kp = " << _param[0] <<  " ki " << _param[1] << " kd " << _param[2] << std::endl;
        // seba check if it's ok or not here....
        //_pid->Init(_param[0], _param[1], _param[2]);
		    _dp[_next_index] *= decrement_step;
      }
      _next_index = (_next_index + 1) % 3;
      std::cout << "\tNext index will be " << _next_index << std::endl;
      _currState = INCREASE_COEFFICIENT;
      break;
    case TRAINING_COMPLETE:
      std::cout << "TRAINING COMPLETE we did it " << _total_train << "PArams " <<
        _param[0] << " " << _param[1] << " " << _param[2] << std::endl;
      _total_train++;
      _best_param = _param;
      _pid->Init(_param[0], _param[1], _param[2]);
      _pid->setTuned(true);
      _currState = TRAINING_INIT;
      break;
    default:
      std::cout << "unknown state " << std::endl;
  }

  _total_error = 0;
  _samples = 1;
}


// void PIDTrainer::Twiddle() {
//     //_param = std::vector<double> (3,0.0);
//     _dp = std::vector<double> (3,0.5);
//     std::cout << "Quando entro nel twiddle quanto valgono i par " << _param[0] << " " << _param[1] << " " << _param[2] << std::endl;
//     //_pid.Init(_param[0], _param[1], _param[2]);
//     double error = CurrentError();
//     if (_best_error == -1)
//         _best_error = error;
//     int it = 0;
//     double init_sum = 0;
//     double sum = std::accumulate(_dp.begin(), _dp.end(), init_sum);
//     while (sum > _threshold) {
//         std::cout << "Iteration " << it << "  best error = " << _best_error << std::endl;
//         for (int i = 0; i < 3 ; ++i) {
//             if (i == 1) {
//                 std::cout << "Ora faccio param 2 e param 1 vale " << _param[0] << std::endl;
//             }
//             std::cout << "\tDentro for indice " << i << " _dp " << _dp[0] << " " << _dp[1] << " " << _dp[2] << std::endl;
//             _param[i] += _dp[i];
//             std::cout << "\tInizializzo primo pid con _params " << _param[0] << " " << _param[1] << " " << _param[2] << std::endl;
//             _pid.Init(_param[0], _param[1], _param[2]);
//             error = CurrentError();
//             std::cout << "\tMah " << error << std::endl;
//             if (error < _best_error) {
//                 std::cout << "\t\tSiamo minori di best error " << std::endl;
//                 _best_error = error;
//                 _dp[i] *= 1.1;
//             } else {
//                 std::cout << "\t\tSiamo maggiori di best error " << std::endl;
//                 _param[i] -= 2 * _dp[i];
//                 std::cout << "\t\tDentro for _dp " << _dp[0] << " " << _dp[1] << " " << _dp[2] << std::endl;
//                 std::cout << "\t\tInizializzo 2 pid con _params " << _param[0] << " " << _param[1] << " " << _param[2] << std::endl;
//                 _pid.Init(_param[0], _param[1], _param[2]);
//                 error = CurrentError();
//                 std::cout << "\t\tMah " << error << std::endl;
//                 if (error < _best_error) {
//                     std::cout << "\t\t\tSiamo minori di best error " << std::endl;
//                     _best_error = error;
//                     _dp[i] *= 1.1;
//                 } else {
//                     std::cout << "\t\t\tSiamo maggiori di best error " << std::endl;
//                     _param[i] += _dp[i];
//                     _dp[i] *= 0.9;
//                 }
//             }
//         }
//         it += 1;
//         sum = std::accumulate(_dp.begin(), _dp.end(), init_sum);
//     }
//     std::cout << "TWIDDLE TERMINATA = " << _param[0] << " " << _param[1] << " " << _param[2] << std::endl;
//     _total_error = 0;
//     _samples = 1;
// }
