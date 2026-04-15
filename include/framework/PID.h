#pragma once

#include "framework/Tuning.h"

namespace framework {

class PIDController {
 public:
  explicit PIDController(const tuning::PIDTune& tune);

  void set(const tuning::PIDTune& tune);
  void rst();

  // Runs one PID step using provided error and fixed loop dt.
  double calc(double error, double dtSec);

  bool ok(double error) const;

  double p() const;
  double i() const;
  double d() const;

 private:
  tuning::PIDTune m_tune;
  double m_integral;
  double m_prevError;
  bool m_hasPrev;

  double m_lastP;
  double m_lastI;
  double m_lastD;
};

}  // namespace framework
