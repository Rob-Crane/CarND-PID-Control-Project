#ifndef PID_H
#define PID_H

#include <vector>

class PID {
 public:
  /**
   * Constructor
   */
  PID(const std::vector<float>& params);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  double UpdateError(double cte);

  /**
   * Update the PID gain values.
   * @param ParamVector of gain values.
   */
  void UpdateGains(const std::vector<float>& params);

 private:
  /**
   * PID Errors
   */
  double p_error_;
  double i_error_;
  double d_error_;

  /**
   * PID Coefficients
   */ 
  double Kp_;
  double Ki_;
  double Kd_;
};

#endif  // PID_H
