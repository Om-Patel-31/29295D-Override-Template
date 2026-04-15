#include "framework/PID.h"

#include <cmath>

namespace framework {

PIDController::PIDController(const tuning::PIDTune& tune)
    : m_tune(tune),
      m_integral(0.0),
      m_prevError(0.0),
      m_hasPrev(false),
      m_lastP(0.0),
      m_lastI(0.0),
      m_lastD(0.0) {}

void PIDController::set(const tuning::PIDTune& tune) {
  m_tune = tune;
  rst();
}

void PIDController::rst() {
  m_integral = 0.0;
  m_prevError = 0.0;
  m_hasPrev = false;
  m_lastP = 0.0;
  m_lastI = 0.0;
  m_lastD = 0.0;
}

double PIDController::calc(double error, double dtSec) {
  // Proportional term reacts immediately to present error.
  m_lastP = m_tune.kP * error;

  // Integral accumulates slowly to remove steady-state bias.
  // Anti-windup clamp prevents integral from growing too large and causing overshoot.
  m_integral += error * dtSec;
  if (m_integral > m_tune.integralClamp) {
    m_integral = m_tune.integralClamp;
  } else if (m_integral < -m_tune.integralClamp) {
    m_integral = -m_tune.integralClamp;
  }
  m_lastI = m_tune.kI * m_integral;

  // Derivative predicts trend by measuring how fast error changes.
  double derivative = 0.0;
  if (m_hasPrev && dtSec > 1e-6) {
    derivative = (error - m_prevError) / dtSec;
  }
  m_lastD = m_tune.kD * derivative;

  m_prevError = error;
  m_hasPrev = true;

  return m_lastP + m_lastI + m_lastD;
}

bool PIDController::ok(double error) const {
  return std::fabs(error) <= m_tune.tolerance;
}

double PIDController::p() const { return m_lastP; }

double PIDController::i() const { return m_lastI; }

double PIDController::d() const { return m_lastD; }

}  // namespace framework
