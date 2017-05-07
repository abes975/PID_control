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
  bool _isTuned;

  /*
  * Coefficients
  */
  double _Ki;
  double _Kp;
  double _Kd;
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
    * Method to access the coefficients
    */
    double getKp() const { return _Kp; }
    double getKi() const { return _Ki; }
    double getKd() const { return _Kd; }
    /*
    * Set coefficient after tuning
    */
    void setNewCoefficients(const std::vector<double>& coeff);
};

#endif /* PID_H */
