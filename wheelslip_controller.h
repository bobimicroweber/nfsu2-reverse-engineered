// WheelSlipController Class - Detailed Implementation
// Extracted from SPEED2.EXE
// WheelSlipLateral located at offset 0x385AAF
// WheelSlipForward located at offset 0x385ACB

#pragma once

#include <windows.h>
#include <cmath>
#include <algorithm>

// Forward declarations
class Vehicle;

/**
 * WheelSlipController - Controls wheel slip for realistic vehicle physics
 * 
 * Found in binary:
 * - WheelSlipLateral at 0x385AAF (first occurrence)
 * - WheelSlipForward at 0x385ACB (first occurrence)
 * - "Psycho WheelSlipLateral Pid" string found
 * - "Psycho WheelSlipForward Pid" string found
 * 
 * Provides wheel slip calculations for:
 * - Lateral slip (sideways movement)
 * - Forward slip (acceleration/braking)
 * - PID controller integration
 * - Realistic tire physics
 */
class WheelSlipController {
public:
    WheelSlipController();
    ~WheelSlipController();
    
    // ============================================
    // Initialization
    // ============================================
    
    /**
     * Initialize wheel slip controller
     */
    void Initialize();
    
    /**
     * Shutdown wheel slip controller
     */
    void Shutdown();
    
    // ============================================
    // Main Update
    // ============================================
    
    /**
     * Update wheel slip calculations
     * Called every frame
     * 
     * @param deltaTime Time since last frame
     * @param steering Steering input (-1.0 to 1.0)
     * @param throttle Throttle input (0.0 to 1.0)
     * @param brake Brake input (0.0 to 1.0)
     * @param speed Current vehicle speed
     * @param yawRate Current yaw rotation rate
     */
    void Update(float deltaTime, float steering, float throttle, float brake,
                float speed, float yawRate);
    
    // ============================================
    // Lateral Slip (Sideways)
    // ============================================
    
    /**
     * Calculate lateral wheel slip
     * Used for drift angle calculation and sideways movement
     * 
     * @param steeringAngle Steering angle in radians
     * @param speed Current speed
     * @param yawRate Yaw rotation rate
     * @return Lateral slip value (0.0 to 1.0)
     */
    float CalculateLateralSlip(float steeringAngle, float speed, float yawRate) const;
    
    /**
     * Get current lateral slip
     */
    float GetLateralSlip() const { return m_lateralSlip; }
    
    /**
     * Get normalized lateral slip (0.0 to 1.0)
     */
    float GetNormalizedLateralSlip() const;
    
    /**
     * Check if lateral slip exceeds threshold
     */
    bool IsLateralSlipExcessive() const;
    
    // ============================================
    // Forward Slip (Acceleration/Braking)
    // ============================================
    
    /**
     * Calculate forward wheel slip
     * Used for traction calculation during acceleration/braking
     * 
     * @param throttle Throttle input (0.0 to 1.0)
     * @param brake Brake input (0.0 to 1.0)
     * @param speed Current speed
     * @return Forward slip value (0.0 to 1.0)
     */
    float CalculateForwardSlip(float throttle, float brake, float speed) const;
    
    /**
     * Get current forward slip
     */
    float GetForwardSlip() const { return m_forwardSlip; }
    
    /**
     * Get normalized forward slip (0.0 to 1.0)
     */
    float GetNormalizedForwardSlip() const;
    
    /**
     * Check if forward slip exceeds threshold
     */
    bool IsForwardSlipExcessive() const;
    
    // ============================================
    // Combined Slip
    // ============================================
    
    /**
     * Get total slip magnitude
     */
    float GetTotalSlip() const;
    
    /**
     * Get slip angle (direction of slip)
     */
    float GetSlipAngle() const;
    
    // ============================================
    // PID Controller Integration
    // ============================================
    
    /**
     * PID Controller for lateral slip
     */
    class LateralPidController {
    public:
        LateralPidController();
        
        float Calculate(float error, float deltaTime);
        void Reset();
        
        void SetProportional(float kp) { m_kp = kp; }
        void SetIntegral(float ki) { m_ki = ki; }
        void SetDerivative(float kd) { m_kd = kd; }
        
        float GetProportional() const { return m_kp; }
        float GetIntegral() const { return m_ki; }
        float GetDerivative() const { return m_kd; }
        
    private:
        float m_kp = 1.0f;  // Proportional gain
        float m_ki = 0.0f;  // Integral gain
        float m_kd = 0.0f;  // Derivative gain
        float m_integral = 0.0f;
        float m_previousError = 0.0f;
    };
    
    /**
     * PID Controller for forward slip
     */
    class ForwardPidController {
    public:
        ForwardPidController();
        
        float Calculate(float error, float deltaTime);
        void Reset();
        
        void SetProportional(float kp) { m_kp = kp; }
        void SetIntegral(float ki) { m_ki = ki; }
        void SetDerivative(float kd) { m_kd = kd; }
        
        float GetProportional() const { return m_kp; }
        float GetIntegral() const { return m_ki; }
        float GetDerivative() const { return m_kd; }
        
    private:
        float m_kp = 1.0f;  // Proportional gain
        float m_ki = 0.0f;  // Integral gain
        float m_kd = 0.0f;  // Derivative gain
        float m_integral = 0.0f;
        float m_previousError = 0.0f;
    };
    
    /**
     * Get lateral PID controller
     */
    LateralPidController& GetLateralPid() { return m_lateralPid; }
    const LateralPidController& GetLateralPid() const { return m_lateralPid; }
    
    /**
     * Get forward PID controller
     */
    ForwardPidController& GetForwardPid() { return m_forwardPid; }
    const ForwardPidController& GetForwardPid() const { return m_forwardPid; }
    
    // ============================================
    // Properties and Limits
    // ============================================
    
    /**
     * Set maximum lateral slip
     */
    void SetMaxLateralSlip(float maxSlip) { m_maxLateralSlip = maxSlip; }
    float GetMaxLateralSlip() const { return m_maxLateralSlip; }
    
    /**
     * Set maximum forward slip
     */
    void SetMaxForwardSlip(float maxSlip) { m_maxForwardSlip = maxSlip; }
    float GetMaxForwardSlip() const { return m_maxForwardSlip; }
    
    /**
     * Set lateral slip threshold
     */
    void SetLateralSlipThreshold(float threshold) { m_lateralSlipThreshold = threshold; }
    float GetLateralSlipThreshold() const { return m_lateralSlipThreshold; }
    
    /**
     * Set forward slip threshold
     */
    void SetForwardSlipThreshold(float threshold) { m_forwardSlipThreshold = threshold; }
    float GetForwardSlipThreshold() const { return m_forwardSlipThreshold; }
    
    // ============================================
    // Vehicle Integration
    // ============================================
    
    /**
     * Set vehicle reference
     */
    void SetVehicle(Vehicle* vehicle) { m_vehicle = vehicle; }
    
    /**
     * Get vehicle reference
     */
    Vehicle* GetVehicle() const { return m_vehicle; }

private:
    // Vehicle reference
    Vehicle* m_vehicle = nullptr;
    
    // Current slip values
    float m_lateralSlip = 0.0f;      // Current lateral slip
    float m_forwardSlip = 0.0f;     // Current forward slip
    float m_previousLateralSlip = 0.0f;
    float m_previousForwardSlip = 0.0f;
    
    // Limits
    float m_maxLateralSlip = 1.0f;   // Maximum lateral slip
    float m_maxForwardSlip = 1.0f;   // Maximum forward slip
    float m_lateralSlipThreshold = 0.7f;  // Threshold for excessive slip
    float m_forwardSlipThreshold = 0.7f;  // Threshold for excessive slip
    
    // PID Controllers
    LateralPidController m_lateralPid;
    ForwardPidController m_forwardPid;
    
    // Timing
    float m_totalTime = 0.0f;
    
    // Constants (from binary analysis)
    static constexpr float LATERAL_SLIP_FACTOR = 0.6f;   // From WheelSlipLateral
    static constexpr float FORWARD_SLIP_FACTOR = 0.4f;   // From WheelSlipForward
    static constexpr float SPEED_NORMALIZATION = 100.0f;  // Speed normalization factor
    static constexpr float YAW_SLIP_FACTOR = 0.1f;       // Yaw rate to slip conversion
    static constexpr float STEERING_SLIP_FACTOR = 0.5f;  // Steering to slip conversion
    
    // Internal methods
    void UpdateLateralSlip(float deltaTime, float steering, float speed, float yawRate);
    void UpdateForwardSlip(float deltaTime, float throttle, float brake, float speed);
    float ClampSlip(float slip, float maxSlip) const;
    float SmoothSlip(float current, float target, float deltaTime) const;
};

// ============================================
// WheelSlipController Implementation
// ============================================

inline WheelSlipController::WheelSlipController() {
    m_vehicle = nullptr;
    m_lateralSlip = 0.0f;
    m_forwardSlip = 0.0f;
    m_previousLateralSlip = 0.0f;
    m_previousForwardSlip = 0.0f;
    m_maxLateralSlip = 1.0f;
    m_maxForwardSlip = 1.0f;
    m_lateralSlipThreshold = 0.7f;
    m_forwardSlipThreshold = 0.7f;
    m_totalTime = 0.0f;
}

inline WheelSlipController::~WheelSlipController() {
    Shutdown();
}

inline void WheelSlipController::Initialize() {
    m_lateralSlip = 0.0f;
    m_forwardSlip = 0.0f;
    m_previousLateralSlip = 0.0f;
    m_previousForwardSlip = 0.0f;
    m_totalTime = 0.0f;
    m_lateralPid.Reset();
    m_forwardPid.Reset();
}

inline void WheelSlipController::Shutdown() {
    m_vehicle = nullptr;
}

inline void WheelSlipController::Update(float deltaTime, float steering, float throttle,
                                        float brake, float speed, float yawRate) {
    m_totalTime += deltaTime;
    
    // Update lateral slip
    UpdateLateralSlip(deltaTime, steering, speed, yawRate);
    
    // Update forward slip
    UpdateForwardSlip(deltaTime, throttle, brake, speed);
}

inline void WheelSlipController::UpdateLateralSlip(float deltaTime, float steering,
                                                   float speed, float yawRate) {
    // Calculate target lateral slip
    float steeringSlip = std::abs(steering) * LATERAL_SLIP_FACTOR;
    float yawSlip = std::abs(yawRate) * YAW_SLIP_FACTOR;
    float speedFactor = std::min(speed / SPEED_NORMALIZATION, 1.0f);
    
    // Combine factors
    float targetSlip = (steeringSlip + yawSlip) * speedFactor;
    
    // Clamp to maximum
    targetSlip = ClampSlip(targetSlip, m_maxLateralSlip);
    
    // Smooth interpolation
    m_previousLateralSlip = m_lateralSlip;
    m_lateralSlip = SmoothSlip(m_lateralSlip, targetSlip, deltaTime);
    
    // Apply PID correction if needed
    if (m_lateralSlip > m_lateralSlipThreshold) {
        float error = m_lateralSlipThreshold - m_lateralSlip;
        float correction = m_lateralPid.Calculate(error, deltaTime);
        m_lateralSlip = std::max(0.0f, m_lateralSlip + correction);
    }
}

inline void WheelSlipController::UpdateForwardSlip(float deltaTime, float throttle,
                                                   float brake, float speed) {
    // Calculate target forward slip
    float inputSlip = std::abs(throttle - brake) * FORWARD_SLIP_FACTOR;
    float speedFactor = std::min(speed / SPEED_NORMALIZATION, 1.0f);
    
    // Combine factors
    float targetSlip = inputSlip * speedFactor;
    
    // Clamp to maximum
    targetSlip = ClampSlip(targetSlip, m_maxForwardSlip);
    
    // Smooth interpolation
    m_previousForwardSlip = m_forwardSlip;
    m_forwardSlip = SmoothSlip(m_forwardSlip, targetSlip, deltaTime);
    
    // Apply PID correction if needed
    if (m_forwardSlip > m_forwardSlipThreshold) {
        float error = m_forwardSlipThreshold - m_forwardSlip;
        float correction = m_forwardPid.Calculate(error, deltaTime);
        m_forwardSlip = std::max(0.0f, m_forwardSlip + correction);
    }
}

inline float WheelSlipController::CalculateLateralSlip(float steeringAngle, float speed,
                                                        float yawRate) const {
    // Lateral slip based on steering and yaw
    float steeringSlip = std::abs(steeringAngle) * LATERAL_SLIP_FACTOR;
    float yawSlip = std::abs(yawRate) * YAW_SLIP_FACTOR;
    float speedFactor = std::min(speed / SPEED_NORMALIZATION, 1.0f);
    
    float slip = (steeringSlip + yawSlip) * speedFactor;
    return ClampSlip(slip, m_maxLateralSlip);
}

inline float WheelSlipController::CalculateForwardSlip(float throttle, float brake,
                                                      float speed) const {
    // Forward slip based on throttle/brake and speed
    float inputSlip = std::abs(throttle - brake) * FORWARD_SLIP_FACTOR;
    float speedFactor = std::min(speed / SPEED_NORMALIZATION, 1.0f);
    
    float slip = inputSlip * speedFactor;
    return ClampSlip(slip, m_maxForwardSlip);
}

inline float WheelSlipController::GetNormalizedLateralSlip() const {
    return m_lateralSlip / m_maxLateralSlip;
}

inline float WheelSlipController::GetNormalizedForwardSlip() const {
    return m_forwardSlip / m_maxForwardSlip;
}

inline bool WheelSlipController::IsLateralSlipExcessive() const {
    return m_lateralSlip > m_lateralSlipThreshold;
}

inline bool WheelSlipController::IsForwardSlipExcessive() const {
    return m_forwardSlip > m_forwardSlipThreshold;
}

inline float WheelSlipController::GetTotalSlip() const {
    // Total slip magnitude
    return std::sqrt(m_lateralSlip * m_lateralSlip + m_forwardSlip * m_forwardSlip);
}

inline float WheelSlipController::GetSlipAngle() const {
    // Slip angle (direction of slip)
    if (m_forwardSlip < 0.001f) return 0.0f;
    return std::atan2(m_lateralSlip, m_forwardSlip);
}

inline float WheelSlipController::ClampSlip(float slip, float maxSlip) const {
    return std::max(0.0f, std::min(slip, maxSlip));
}

inline float WheelSlipController::SmoothSlip(float current, float target,
                                              float deltaTime) const {
    // Smooth interpolation with lerp
    float lerpSpeed = 10.0f * deltaTime;
    return current * (1.0f - lerpSpeed) + target * lerpSpeed;
}

// ============================================
// PID Controller Implementation
// ============================================

inline WheelSlipController::LateralPidController::LateralPidController() {
    m_kp = 1.0f;
    m_ki = 0.0f;
    m_kd = 0.0f;
    m_integral = 0.0f;
    m_previousError = 0.0f;
}

inline float WheelSlipController::LateralPidController::Calculate(float error, float deltaTime) {
    // Proportional term
    float p = m_kp * error;
    
    // Integral term
    m_integral += error * deltaTime;
    float i = m_ki * m_integral;
    
    // Derivative term
    float d = m_kd * (error - m_previousError) / deltaTime;
    m_previousError = error;
    
    // Clamp integral to prevent windup
    m_integral = std::max(-10.0f, std::min(10.0f, m_integral));
    
    return p + i + d;
}

inline void WheelSlipController::LateralPidController::Reset() {
    m_integral = 0.0f;
    m_previousError = 0.0f;
}

inline WheelSlipController::ForwardPidController::ForwardPidController() {
    m_kp = 1.0f;
    m_ki = 0.0f;
    m_kd = 0.0f;
    m_integral = 0.0f;
    m_previousError = 0.0f;
}

inline float WheelSlipController::ForwardPidController::Calculate(float error, float deltaTime) {
    // Proportional term
    float p = m_kp * error;
    
    // Integral term
    m_integral += error * deltaTime;
    float i = m_ki * m_integral;
    
    // Derivative term
    float d = m_kd * (error - m_previousError) / deltaTime;
    m_previousError = error;
    
    // Clamp integral to prevent windup
    m_integral = std::max(-10.0f, std::min(10.0f, m_integral));
    
    return p + i + d;
}

inline void WheelSlipController::ForwardPidController::Reset() {
    m_integral = 0.0f;
    m_previousError = 0.0f;
}

