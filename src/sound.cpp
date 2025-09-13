#include "sound.hpp"

namespace {
  struct ActiveTone { int freq=0; unsigned long endMs=0; bool active=false; };
  ActiveTone current;

  struct SeqStep { int freq; int dur; int gap; }; // gap after tone
  struct Sequence { const SeqStep* steps; size_t count; };

  // Define reusable sequences (official / more polished feel)
  // Short gaps provide a cohesive "UI" audio language.
  const SeqStep startupSteps[] = {
    {1000,120,40}, {1300,120,40}, {1600,180,0}
  };
  const SeqStep calibrateStartSteps[] = {
    {900,140,50},{1100,140,0}
  };
  const SeqStep calibrateDoneSteps[] = {
    {1400,120,40},{1700,160,0}
  };
  const SeqStep expProjectileStart[] = {
    {1200,100,40},{1500,100,40},{1800,140,0}
  };
  const SeqStep expPendulumStart[] = {
    {1000,100,40},{1400,100,40},{1700,140,0}
  };
  const SeqStep pendulumMeasureStartSeq[] = {
    {1500,160,50},{1800,100,0}
  };
  const SeqStep expFreefallStart[] = {
    {900,100,40},{1200,100,40},{1500,140,0}
  };
  const SeqStep expFrictionStart[] = {
    {1100,100,40},{1300,100,40},{1600,140,0}
  };
  const SeqStep projectileThrowSeq[] = {
    {1800,160,60},{2200,100,0}
  };
  const SeqStep projectileFreefallSeq[] = {
    {1000,140,40},{1200,100,0}
  };
  const SeqStep pendulumPeakSeq[] = {
    {2000,60,0}
  };
  const SeqStep freefallStartSeq[] = {
    {1600,100,40},{1400,80,0}
  };
  const SeqStep freefallImpactSeq[] = {
    {900,140,60},{1200,140,60},{1500,180,0}
  };
  const SeqStep frictionSlipSeq[] = {
    {1100,140,60},{1400,140,60},{1700,180,0}
  };
  const SeqStep doneSeq[] = {
    {800,140,70},{1000,140,70},{1200,200,0}
  };

  struct RunningSequence {
    const SeqStep* steps=nullptr; size_t count=0; size_t index=0; unsigned long nextStart=0; bool active=false; } seq;

  void startSequence(const Sequence& s) {
    seq.steps = s.steps;
    seq.count = s.count;
    seq.index = 0;
    seq.nextStart = 0; // fire immediately
    seq.active = true;
  }

  void stopTone() {
    if (current.active && millis() >= current.endMs) {
      M5.Speaker.stop();
      current.active = false;
    }
  }

  void processSequence() {
    if (!seq.active) return;
    unsigned long now = millis();
    if (current.active) return; // wait tone end first
    if (now < seq.nextStart) return; // wait gap
    if (seq.index >= seq.count) { seq.active = false; return; }
    const auto& step = seq.steps[seq.index];
    M5.Speaker.setVolume(255);
    M5.Speaker.tone(step.freq);
    current.freq = step.freq;
    current.endMs = now + step.dur;
    current.active = true;
    seq.nextStart = current.endMs + step.gap;
    seq.index++;
  }

  // Lookup table mapping events to sequences
  Sequence mapEvent(Sound::Event e) {
    switch(e) {
      case Sound::Event::Startup: return {startupSteps, sizeof(startupSteps)/sizeof(SeqStep)};
      case Sound::Event::CalibrateStart: return {calibrateStartSteps, sizeof(calibrateStartSteps)/sizeof(SeqStep)};
      case Sound::Event::CalibrateDone: return {calibrateDoneSteps, sizeof(calibrateDoneSteps)/sizeof(SeqStep)};
      case Sound::Event::ExperimentStartProjectile: return {expProjectileStart, sizeof(expProjectileStart)/sizeof(SeqStep)};
      case Sound::Event::ExperimentStartPendulum: return {expPendulumStart, sizeof(expPendulumStart)/sizeof(SeqStep)};
  case Sound::Event::PendulumMeasureStart: return {pendulumMeasureStartSeq, sizeof(pendulumMeasureStartSeq)/sizeof(SeqStep)};
      case Sound::Event::ExperimentStartFreefall: return {expFreefallStart, sizeof(expFreefallStart)/sizeof(SeqStep)};
      case Sound::Event::ExperimentStartFriction: return {expFrictionStart, sizeof(expFrictionStart)/sizeof(SeqStep)};
      case Sound::Event::ProjectileThrow: return {projectileThrowSeq, sizeof(projectileThrowSeq)/sizeof(SeqStep)};
      case Sound::Event::ProjectileFreefall: return {projectileFreefallSeq, sizeof(projectileFreefallSeq)/sizeof(SeqStep)};
      case Sound::Event::PendulumPeak: return {pendulumPeakSeq, sizeof(pendulumPeakSeq)/sizeof(SeqStep)};
      case Sound::Event::FreefallStart: return {freefallStartSeq, sizeof(freefallStartSeq)/sizeof(SeqStep)};
      case Sound::Event::FreefallImpact: return {freefallImpactSeq, sizeof(freefallImpactSeq)/sizeof(SeqStep)};
      case Sound::Event::FrictionSlip: return {frictionSlipSeq, sizeof(frictionSlipSeq)/sizeof(SeqStep)};
      case Sound::Event::ExperimentDone: return {doneSeq, sizeof(doneSeq)/sizeof(SeqStep)};
      default: return {nullptr,0};
    }
  }
}

namespace Sound {
  void begin() {
    // nothing special now; placeholder for future mixer
  }

  void update() {
    stopTone();
    processSequence();
  }

  void playTone(int freq, int durationMs) {
    if (freq <= 0 || durationMs <= 0) return;
    M5.Speaker.setVolume(255);
    M5.Speaker.tone(freq);
    current.freq = freq;
    current.endMs = millis() + (unsigned long)durationMs;
    current.active = true;
  }

  void trigger(Event e) {
    auto seqDef = mapEvent(e);
    if (!seqDef.steps || seqDef.count == 0) return;
    startSequence({seqDef.steps, seqDef.count});
  }
}
