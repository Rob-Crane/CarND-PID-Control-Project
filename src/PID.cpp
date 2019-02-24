#include "PID.h"

PID::PID(const std::vector<float>& params)
    : Kp_(params[0]),
      Kd_(params[1]),
      Ki_(params[2]),
      i_error_(0.0),
      d_error_(0.0) {}

double PID::UpdateError(double cte) {
  i_error_ += cte;
  d_error_ = cte - p_error_;
  p_error_ = cte;

  return Kp_ * p_error_ + Ki_ * i_error_ + Kd_ * d_error_;
}

void PID::UpdateGains(const std::vector<float>& params) {
  Kp_ = params[0];
  Kd_ = params[1];
  Ki_ = params[2];
}
