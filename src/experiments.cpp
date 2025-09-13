// experiments.cpp - تنفيذ منطق التجارب بعد فصلها عن main.cpp
#include <M5Unified.h>
#include "filters.hpp"
#include "experiments.hpp"

// الجاذبية القياسية (معرّفة في main.cpp أيضاً كـ extern)
extern KalmanFilter axFilter;
extern KalmanFilter ayFilter;
extern KalmanFilter azFilter;
// (أزيل playSound القديم بعد اعتماد نظام Sound الحدثي)
#include "sound.hpp"

// الثوابت العامة
const float GRAVITY_CONST = 9.80665f; // مكررة هنا كتعريف واحد (المعلن في الهيدر extern)

// الحالة العامة
ExperimentType activeExperiment = NONE;
ExperimentState experimentState = IDLE;

// -----------------------------
// متغيرات المقذوفات
// -----------------------------
float proj_g0 = 0.0f, proj_mass = 0.5f, proj_velocity = 0.0f, proj_height = 0.0f;
unsigned long proj_time_us = 0;
float proj_V0 = 0.0f, proj_T = 0.0f, proj_h_max = 0.0f, proj_g_exp = 0.0f, proj_F_max = 0.0f;
float proj_angle_deg = 45.0f; 
bool proj_freefall_started = false;
int  proj_landing_samples_count = 0;
float proj_g_sum = 0.0f; 
int  proj_g_samples = 0;
const float PROJ_THROW_DETECT_THRESHOLD = 2.5f;
const float PROJ_FREEFALL_DETECT_THRESHOLD = 1.2f;
const float PROJ_LANDING_DETECT_THRESHOLD = 0.2f;
const int   PROJ_LANDING_SAMPLES_REQUIRED = 20;

// -----------------------------
// متغيرات البندول
// -----------------------------
float pend_period = 0.0f, pend_frequency = 0.0f, pend_string_length = 0.5f, pend_g_exp = 0.0f;
int   pend_oscillations_to_measure = 10;
int   pend_oscillation_count = 0;
unsigned long pend_startTime = 0;
bool  pend_isSwinging = false;
float pend_g0_y = 0.0f;
const float PEND_SWING_THRESHOLD = 0.2f;

// -----------------------------
// متغيرات السقوط الحر
// -----------------------------
float freefall_distance = 1.0f, freefall_time = 0.0f, freefall_g_exp = 0.0f;
unsigned long freefall_start_time = 0;
const float FREEFALL_DETECT_THRESHOLD = 0.3f; 
const float FREEFALL_IMPACT_THRESHOLD = 3.5f; 

// -----------------------------
// متغيرات الاحتكاك
// -----------------------------
float fric_g0_x = 0.0f, fric_g0_z = 0.0f;
float fric_current_angle = 0.0f, fric_critical_angle = 0.0f, fric_mu = 0.0f;
float fric_zero_angle = 0.0f;
const float FRIC_SLIP_THRESHOLD = 0.5f;

// ------------------------------------------------------------------
// إعادة ضبط
// ------------------------------------------------------------------
void resetExperimentData() {
    proj_velocity = 0.0f; proj_height = 0.0f; proj_V0 = 0.0f; proj_T = 0.0f; proj_h_max = 0.0f; proj_g_exp = 0.0f; proj_F_max = 0.0f;
    proj_freefall_started = false; proj_landing_samples_count = 0; proj_g_sum = 0.0f; proj_g_samples = 0;
    pend_period = 0.0f; pend_frequency = 0.0f; pend_oscillation_count = 0; pend_isSwinging = false; pend_g_exp = 0.0f;
    freefall_time = 0.0f; freefall_g_exp = 0.0f;
    fric_current_angle = 0.0f; fric_critical_angle = 0.0f; fric_mu = 0.0f;
}

// ------------------------------------------------------------------
// منطق المقذوفات
// ------------------------------------------------------------------
void projectileController() {
    static unsigned long last_update_us = 0;
    if (experimentState == IDLE || experimentState == DONE) return;

    float ax_raw, ay_raw, az_raw;
    M5.Imu.getAccelData(&ax_raw, &ay_raw, &az_raw);
    float ax = axFilter.update(ax_raw);
    float ay = ayFilter.update(ay_raw);
    float az = azFilter.update(az_raw);

    float net_accel_g = az - proj_g0;
    float vertical_accel = net_accel_g * GRAVITY_CONST;

    if (experimentState == WAITING) {
        if (vertical_accel > PROJ_THROW_DETECT_THRESHOLD * GRAVITY_CONST) {
            experimentState = RUNNING;
            last_update_us = micros();
            Sound::trigger(Sound::Event::ProjectileThrow);
            M5.Display.fillScreen(ORANGE); M5.Display.setCursor(0, 80); M5.Display.println("THROW DETECTED!");
        }
        return;
    }

    if (experimentState == RUNNING) {
        unsigned long current_us = micros();
        float dt = (current_us - last_update_us) / 1000000.0f;
        last_update_us = current_us;

        if (!proj_freefall_started) {
            proj_velocity += vertical_accel * dt;
            float current_force = proj_mass * fabs(vertical_accel);
            if (current_force > proj_F_max) proj_F_max = current_force;

            if (fabs(net_accel_g) < PROJ_FREEFALL_DETECT_THRESHOLD) {
                proj_freefall_started = true;
                proj_V0 = proj_velocity;
                proj_time_us = current_us;
                proj_height = 0.0f; proj_h_max = 0.0f;
                Sound::trigger(Sound::Event::ProjectileFreefall);
                M5.Display.fillScreen(BLUE); M5.Display.setCursor(0, 80); M5.Display.println("FREEFALL...");
            }
        } else {
            proj_velocity += (net_accel_g * GRAVITY_CONST) * dt;
            proj_height += proj_velocity * dt;
            if (proj_height > proj_h_max) proj_h_max = proj_height; // الإصلاح
            proj_g_sum += fabs(net_accel_g);
            proj_g_samples++;
        }

        if (proj_freefall_started && fabs(net_accel_g) < PROJ_LANDING_DETECT_THRESHOLD) {
            proj_landing_samples_count++;
        } else {
            proj_landing_samples_count = 0;
        }

        if (proj_landing_samples_count >= PROJ_LANDING_SAMPLES_REQUIRED) {
            experimentState = DONE;
            proj_T = (current_us - proj_time_us) / 1000000.0f;
            proj_g_exp = (proj_g_samples > 0) ? (proj_g_sum / proj_g_samples) * GRAVITY_CONST : 0;
            Sound::trigger(Sound::Event::ExperimentDone);
            M5.Display.fillScreen(DARKGREEN); M5.Display.setCursor(0, 80); M5.Display.println("DONE! \nCheck browser.");
        }
    }
}

// ------------------------------------------------------------------
// منطق البندول
// ------------------------------------------------------------------
void pendulumController() {
    if (experimentState == IDLE || experimentState == DONE) return;
    static float last_smoothed_g_y = 0.0f; static bool was_increasing = false; static unsigned long last_peak_time = 0;

    float ax_raw, ay_raw, az_raw; M5.Imu.getAccelData(&ax_raw, &ay_raw, &az_raw);
    float ax = axFilter.update(ax_raw); (void)ax; // غير مستخدم مباشرة الآن
    float ay = ayFilter.update(ay_raw); float az = azFilter.update(az_raw); (void)az;
    float current_g_y = ay - pend_g0_y;

    if (experimentState == WAITING) {
        if (fabs(current_g_y) > PEND_SWING_THRESHOLD) {
            experimentState = RUNNING;
            last_smoothed_g_y = 0; was_increasing = false; last_peak_time = 0;
            M5.Display.fillScreen(ORANGE); M5.Display.setCursor(0, 80); M5.Display.println("Measuring...");
            Sound::trigger(Sound::Event::PendulumMeasureStart);
        }
        return;
    }

    if (experimentState == RUNNING) {
        float smoothed_g_y = (current_g_y * 0.4f) + (last_smoothed_g_y * 0.6f);
        bool is_increasing = smoothed_g_y > last_smoothed_g_y;
        if (millis() - last_peak_time > 250) {
            if ((was_increasing && !is_increasing && smoothed_g_y > PEND_SWING_THRESHOLD) || (!was_increasing && is_increasing && smoothed_g_y < -PEND_SWING_THRESHOLD)) {
                Sound::trigger(Sound::Event::PendulumPeak); last_peak_time = millis();
            }
        }
        was_increasing = is_increasing; last_smoothed_g_y = smoothed_g_y;

        bool previousState = pend_isSwinging; pend_isSwinging = current_g_y > 0;
        if (previousState != pend_isSwinging) {
            if (pend_oscillation_count == 0) pend_startTime = millis();
            pend_oscillation_count++;
            if (pend_oscillation_count >= pend_oscillations_to_measure * 2) {
                unsigned long endTime = millis(); float totalTime = (endTime - pend_startTime) / 1000.0f;
                pend_period = totalTime / pend_oscillations_to_measure;
                if (pend_period > 0) { pend_frequency = 1.0f / pend_period; pend_g_exp = (4.0f * PI * PI * pend_string_length) / (pend_period * pend_period); } else { pend_frequency = 0; pend_g_exp = 0; }
                experimentState = DONE;
                Sound::trigger(Sound::Event::ExperimentDone);
                M5.Display.fillScreen(DARKGREEN); M5.Display.setCursor(0, 80); M5.Display.println("DONE! \nCheck browser.");
            }
        }
    }
}

// ------------------------------------------------------------------
// منطق السقوط الحر
// ------------------------------------------------------------------
void freefallController() {
    if (experimentState == IDLE || experimentState == DONE) return;
    float ax_raw, ay_raw, az_raw; M5.Imu.getAccelData(&ax_raw, &ay_raw, &az_raw);
    float ax = axFilter.update(ax_raw); float ay = ayFilter.update(ay_raw); float az = azFilter.update(az_raw);
    float total_accel_mag = sqrtf(ax*ax + ay*ay + az*az);
    if (experimentState == WAITING) {
        if (total_accel_mag < FREEFALL_DETECT_THRESHOLD) {
            experimentState = RUNNING; freefall_start_time = millis();
            M5.Display.fillScreen(ORANGE); M5.Display.setCursor(0, 80); M5.Display.println("FALLING...");
            Sound::trigger(Sound::Event::FreefallStart);
        }
        return;
    }
    if (experimentState == RUNNING) {
        if (total_accel_mag > FREEFALL_IMPACT_THRESHOLD) {
            unsigned long endTime = millis(); freefall_time = (endTime - freefall_start_time) / 1000.0f;
            if (freefall_time > 0.05f) freefall_g_exp = (2.0f * freefall_distance) / (freefall_time * freefall_time); else { freefall_time = 0; freefall_g_exp = 0; }
            experimentState = DONE;
            Sound::trigger(Sound::Event::FreefallImpact);
            M5.Display.fillScreen(DARKGREEN); M5.Display.setCursor(0, 80); M5.Display.println("DONE! \nCheck browser.");
        }
    }
}

// ------------------------------------------------------------------
// منطق الاحتكاك
// ------------------------------------------------------------------
void frictionController() {
    if (experimentState == IDLE || experimentState == DONE) return;
    float ax_raw, ay_raw, az_raw; M5.Imu.getAccelData(&ax_raw, &ay_raw, &az_raw);
    float ax = axFilter.update(ax_raw); float az = azFilter.update(az_raw); (void)ay_raw; (void)ayFilter; // المحور Y غير مستخدم هنا
    float pitch = atan2f(-ax, az) * 180.0f / PI; fric_current_angle = pitch;
    if (fabs(ax - fric_g0_x) > FRIC_SLIP_THRESHOLD) {
        if (experimentState == RUNNING) {
            fric_critical_angle = fric_current_angle; fric_mu = tanf(fric_critical_angle * PI / 180.0f); experimentState = DONE;
            Sound::trigger(Sound::Event::FrictionSlip);
            Sound::trigger(Sound::Event::ExperimentDone);
            M5.Display.fillScreen(DARKGREEN); M5.Display.setCursor(0, 80); M5.Display.println("DONE! \nCheck browser.");
        }
    }
}
