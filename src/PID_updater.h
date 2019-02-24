#include <sstream>
#include <string>
#include <vector>

using std::string;

using ParamVector = std::vector<float>;

string param_str(const ParamVector& p) {
  std::stringstream ss;
  ss << "[" << p[0] << ", " << p[1] << ", " << p[2] << "]";
  return ss.str();
}

class PIDUpdater {
 public:
  PIDUpdater(const ParamVector& initial_p, const ParamVector& initial_dp,
             float upscale, float downscale, unsigned int i);

  void good_outcome();
  void bad_outcome();
  string desc();
  string state_desc();

  const ParamVector& p() { return p_; }

 private:
  ParamVector p_;
  ParamVector dp_;
  double upscale_;
  double downscale_;
  // i is index of parameter currently being evaluated.
  unsigned int i_;
  enum class State { kTryAdd, kTrySub };
  State state_;
};

PIDUpdater::PIDUpdater(const ParamVector& initial_p,
                       const ParamVector& initial_dp, float upscale,
                       float downscale, unsigned int i)
    : p_(initial_p),
      dp_(initial_dp),
      upscale_(upscale),
      downscale_(downscale),
      i_(i),
      state_(State::kTryAdd) {}

void PIDUpdater::good_outcome() {
  dp_[i_] *= upscale_;
  if (state_ == State::kTrySub) {
    state_ = State::kTryAdd;
  }
  i_ = (i_+1)%3;
  p_[i_] += dp_[i_];
}
void PIDUpdater::bad_outcome() {
  if (state_ == State::kTryAdd) {
    state_ = State::kTrySub;
    p_[i_] -= 2 * dp_[i_];
  } else {
    p_[i_] += dp_[i_];
    dp_[i_] *= downscale_;
    state_ = State::kTryAdd;
    i_ = (i_+1)%3;
    p_[i_] += dp_[i_];
  }
}

string PIDUpdater::desc() {
  std::stringstream ss;
  ss << "p[" << i_ << "]: ";
  if (state_ == State::kTrySub)
    ss << "-";
  else
    ss << "+";
  ss << dp_[i_];
  return ss.str();
}

string PIDUpdater::state_desc() {
  std::stringstream ss;
  ss << "P[" << p_[0] << ", " << p_[1] << ", " << p_[2] << "] dP[" << dp_[0]
     << ", " << dp_[1] << ", " << dp_[2] << "]";
  return ss.str();
}
