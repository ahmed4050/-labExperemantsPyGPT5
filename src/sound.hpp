#pragma once
#include <M5Unified.h>

namespace Sound {
  enum class Event {
    Startup,
    CalibrateStart,
    CalibrateDone,
    ExperimentStartProjectile,
    ExperimentStartPendulum,
    ExperimentStartFreefall,
    ExperimentStartFriction,
    PendulumMeasureStart,
    ProjectileThrow,
    ProjectileFreefall,
    ExperimentDone,
    PendulumPeak,
    FreefallStart,
    FreefallImpact,
    FrictionSlip
  };

  void begin();
  void update();
  void playTone(int freq, int durationMs); // low-level (still non-blocking)
  void trigger(Event e);
}
