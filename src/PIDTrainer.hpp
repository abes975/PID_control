#ifndef PID_TRAINER_H
#define PID_TRAINER_H
#include "PID.h"

#include <vector>


class PIDTrainer {
  public:
    PIDTrainer(double threshold): _dp(3,1.0), _p(3,0.0), _threshold(threshold),
      _resetTreshold(2.5), _needReset(false) {};
    PIDTrainer() : _dp(3,1.0), _p(3,0.0), _threshold(0.2), _resetTreshold(2.5),
      _needReset(false) {};
    PIDTrainer(PID& p, double threshold, double resetTreshold);
    enum state { INIT,
                 COMPUTE_BEST_ERROR,
                 CHECK_FINISHED,
                 EVALUATE_ERROR,
                 NEED_MORE_SAMPLES,
                 NEED_MORE_SAMPLES_2,
                 TRAINING_COMPLETE };
    void TuneParameters(double cte);
    void saveSamples(double sample);
    void twiddle(double cte, int sample_limit);
    const std::vector<double>& dumpCoefficient() const;
    bool needReset() const;
    PIDTrainer::state getState() const;
    void incError(double error);
  private:
    std::vector<double> _samples;
    std::vector<double> _dp;
    std::vector<double> _p;
    double _threshold;
    double _resetTreshold;
    bool _needReset;
    int _next_index;
    int _proc_samples;
    PIDTrainer::state _currState;
    PID _pid;
    double _best_error;
    double _error;
    void UpdateErrors(PID &p);
};


#endif
