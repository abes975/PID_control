#ifndef PID_TRAINER_H
#define PID_TRAINER_H
#include "PID.h"

#include <vector>


class PIDTrainer {
  public:
    PIDTrainer(double threshold): _dp(3,1.0), _param(3,0.0), _threshold(threshold) {};
    PIDTrainer() : _dp(3,1.0), _param(3,0.0), _threshold(0.2) {};
    PIDTrainer(PID* p, double threshold, double resetTreshold);
    enum state { TRAINING_INIT,
                 INIT,
                 INCREASE_COEFFICIENT,
                 EVALUATE_ERROR_AFTER_INCREASE,
                 EVALUATE_ERROR_AFTER_DECREASE,
                 TRAINING_COMPLETE };
    void TuneParameters();
    //void Twiddle();
    void UpdateError(double cte);
    const std::vector<double>& dumpCoefficient() const;
    PIDTrainer::state getState() const;
    double increment_step;
    double decrement_step;
  private:
    std::vector<double> _dp;
    std::vector<double> _param;
    std::vector<double> _best_param;
    double _threshold;
    int _next_index;
    int _samples;
    PIDTrainer::state _currState;
    PID* _pid;
    double _best_error;
    double _total_error;
    double CurrentError();
    int _total_train;
};


#endif
