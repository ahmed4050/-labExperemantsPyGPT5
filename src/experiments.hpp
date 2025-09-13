// experiments.hpp - فصل منطق التجارب والمتغيرات الخاصة بها
#pragma once

#include <Arduino.h>

// الجاذبية القياسية (تستخدم في الحسابات)
extern const float GRAVITY_CONST;

// أنواع التجارب وحالاتها
enum ExperimentType { NONE, PROJECTILE, PENDULUM, FREEFALL, FRICTION };
enum ExperimentState { IDLE, WAITING, RUNNING, DONE };

// الحالة العامة الجارية
extern ExperimentType activeExperiment;
extern ExperimentState experimentState;

// -----------------------------
// متغيرات تجربة المقذوفات
// -----------------------------
extern float proj_g0, proj_mass, proj_velocity, proj_height;
extern unsigned long proj_time_us;
extern float proj_V0, proj_T, proj_h_max, proj_g_exp, proj_F_max;
extern float proj_angle_deg; 
extern bool proj_freefall_started;
extern int  proj_landing_samples_count;
extern float proj_g_sum; 
extern int  proj_g_samples;
extern const float PROJ_THROW_DETECT_THRESHOLD, PROJ_FREEFALL_DETECT_THRESHOLD, PROJ_LANDING_DETECT_THRESHOLD;
extern const int   PROJ_LANDING_SAMPLES_REQUIRED;

// -----------------------------
// متغيرات تجربة البندول
// -----------------------------
extern float pend_period, pend_frequency, pend_string_length, pend_g_exp;
extern int   pend_oscillations_to_measure;
extern int   pend_oscillation_count;
extern unsigned long pend_startTime;
extern bool  pend_isSwinging;
extern float pend_g0_y; 
extern const float PEND_SWING_THRESHOLD;

// -----------------------------
// متغيرات تجربة السقوط الحر
// -----------------------------
extern float freefall_distance, freefall_time, freefall_g_exp;
extern unsigned long freefall_start_time;
extern const float FREEFALL_DETECT_THRESHOLD; 
extern const float FREEFALL_IMPACT_THRESHOLD; 

// -----------------------------
// متغيرات تجربة الاحتكاك
// -----------------------------
extern float fric_g0_x, fric_g0_z;
extern float fric_current_angle, fric_critical_angle, fric_mu;
extern float fric_zero_angle;
extern const float FRIC_SLIP_THRESHOLD;

// -----------------------------
// واجهة الدوال
// -----------------------------
void projectileController();
void pendulumController();
void freefallController();
void frictionController();

// إعادة ضبط المتغيرات الخاصة بالتجارب فقط
void resetExperimentData();
