#ifndef PID_TRAINER_H
#define PID_TRAINER_H
#include "PID.h"

#include <vector>


class PIDTrainer {
  public:
    PIDTrainer(PID* p, double threshold);
    enum state { TRAINING_INIT,
                 EVALUATE_ERROR_AFTER_INCREASE,
                 EVALUATE_ERROR_AFTER_DECREASE,
                 TRAINING_COMPLETE };
    void TuneParameters();
    void UpdateError(double cte);
    const std::vector<double>& dumpCoefficient() const;
    double GetBestError() const { return _best_error; }
    std::vector<double> increment_step;
    std::vector<double> decrement_step;
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
