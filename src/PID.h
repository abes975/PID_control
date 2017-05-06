#ifndef PID_H
#define PID_H
#include <vector>

class PID {
private:
  /*
  * Errors
  */
  double _p_error;
  double _i_error;
  double _d_error;
  double _prev_error;
  double _sum_error;
  std::vector<double> _dp;
  std::vector<double> _p;
  bool _isTuned;

  /*
  * Coefficients
  */
  double _Ki;
  double _Kp;
  double _Kd;
  void UpdateError(std::vector<double> ctes);
  double TotalError(std::vector<double>& p);
  std::vector<double> saveErrors();
  void restoreErrors(std::vector<double>& err);
  public:
    /*
    * Constructor
    */
    PID();

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID.
    */
    void Init(double Kp, double Ki, double Kd);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();


    /*
    * Check if we have already tuned parameters
    */
    bool isTuned() const;

    /*
    * Tune parameters
    */
    void twiddle(double threshold, std::vector<double> ctes);

    void printErrors();

};

#endif /* PID_H */
