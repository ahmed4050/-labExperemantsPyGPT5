#pragma once

// Kalman 1D simple filter used to smooth accelerometer readings.
// Tunable parameters Q (process noise) and R (measurement noise).
class KalmanFilter {
private:
    float Q; // Process noise
    float R; // Measurement noise
    float P; // Estimation error covariance
    float X; // Estimated value
    float K; // Kalman gain
public:
    KalmanFilter(float q = 0.01f, float r = 0.1f, float initial = 0.0f)
        : Q(q), R(r), P(0.1f), X(initial), K(0.0f) {}

    inline void setTuning(float q, float r) { Q = q; R = r; }
    inline float value() const { return X; }
    inline void reset(float initial = 0.0f) { X = initial; P = 0.1f; }

    float update(float measurement) {
        // Compute gain
        K = P / (P + R);
        // Update estimate
        X = X + K * (measurement - X);
        // Update error covariance
        P = (1 - K) * P + Q;
        return X;
    }
};
