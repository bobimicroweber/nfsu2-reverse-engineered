// Drift Classes
// Extracted from SPEED2.EXE
// Drift driving mechanics for Need for Speed Underground 2

#pragma once

#include <windows.h>
#include <cmath>
#include <algorithm>
#include "carcontroller_enhanced.h"
#include "realcontroller_logic_detailed.h"

// Forward declarations
class Vehicle;

/**
 * DriftDriver - Advanced drift driving controller
 * 
 * Found in binary at offset 0x3844A4, 0x385B2C
 * Provides drift-specific driving mechanics
 */
class DriftDriver {
public:
    DriftDriver();
    ~DriftDriver();
    
    // ============================================
    // Initialization
    // ============================================
    
    void Initialize();
    void Shutdown();
    
    // ============================================
    // Drift Control
    // ============================================
    
    /**
     * Update drift driver
     * Called every frame
     */
    void Update(float deltaTime);
    
    /**
     * Apply drift input
     * @param throttle Throttle input (0.0 to 1.0)
     * @param brake Brake input (0.0 to 1.0)
     * @param steering Steering input (-1.0 to 1.0)
     * @param handbrake Handbrake input (0.0 to 1.0)
     */
    void ApplyDriftInput(float throttle, float brake, float steering, float handbrake);
    
    /**
     * Calculate drift angle
     * Returns current drift angle in radians
     */
    float CalculateDriftAngle() const;
    
    /**
     * Get drift score
     * Returns current drift score/points
     */
    float GetDriftScore() const { return m_driftScore; }
    
    /**
     * Reset drift score
     */
    void ResetDriftScore() { m_driftScore = 0.0f; }
    
    // ============================================
    // Drift State
    // ============================================
    
    enum class DriftState {
        None,
        Initiating,
        Sustaining,
        Exiting,
        Spinning
    };
    
    DriftState GetState() const { return m_state; }
    bool IsDrifting() const { return m_state != DriftState::None; }
    
    // ============================================
    // Snapshot System
    // ============================================
    
    /**
     * DoSnapshot - Take a snapshot of drift state
     * Found in binary as "DriftDriver::DoSnapshot"
     */
    void DoSnapshot();
    
    /**
     * Get snapshot data
     */
    struct Snapshot {
        float driftAngle;
        float speed;
        float throttle;
        float steering;
        float timestamp;
    };
    
    const Snapshot& GetLastSnapshot() const { return m_lastSnapshot; }
    
    // ============================================
    // Properties
    // ============================================
    
    void SetVehicle(Vehicle* vehicle) { m_vehicle = vehicle; }
    Vehicle* GetVehicle() const { return m_vehicle; }
    
    float GetDriftAngle() const { return m_driftAngle; }
    float GetSlipAngle() const { return m_slipAngle; }
    float GetYawRate() const { return m_yawRate; }

private:
    // Vehicle reference
    Vehicle* m_vehicle = nullptr;
    
    // Drift state
    DriftState m_state = DriftState::None;
    
    // Drift metrics
    float m_driftAngle = 0.0f;      // Current drift angle
    float m_slipAngle = 0.0f;       // Slip angle
    float m_yawRate = 0.0f;         // Yaw rotation rate
    float m_driftScore = 0.0f;      // Accumulated drift score
    
    // Inputs
    float m_throttle = 0.0f;
    float m_brake = 0.0f;
    float m_steering = 0.0f;
    float m_handbrake = 0.0f;
    
    // Physics
    float m_currentSpeed = 0.0f;
    float m_previousSpeed = 0.0f;
    
    // Snapshot
    Snapshot m_lastSnapshot;
    
    // Constants (from binary analysis)
    static constexpr float MIN_DRIFT_SPEED = 30.0f;      // Minimum speed to drift
    static constexpr float MAX_DRIFT_ANGLE = 1.57f;      // ~90 degrees
    static constexpr float DRIFT_ANGLE_THRESHOLD = 0.3f; // Angle to consider drifting
    static constexpr float SLIP_MULTIPLIER = 0.6f;       // From binary: WheelSlip constants
    static constexpr float YAW_DAMPING = 0.85f;          // Yaw rate damping
    
    // Internal methods
    void UpdateDriftState(float deltaTime);
    void UpdateDriftAngle(float deltaTime);
    void UpdateYawRate(float deltaTime);
    void CalculateDriftScore(float deltaTime);
    bool CheckDriftConditions() const;
};

/**
 * WheelSlipController - Controls wheel slip for drifting
 * 
 * Found in binary:
 * - WheelSlipLateral at 0x385AAF
 * - WheelSlipForward at 0x385ACB
 */
class WheelSlipController {
public:
    WheelSlipController();
    ~WheelSlipController();
    
    // ============================================
    // Slip Calculation
    // ============================================
    
    /**
     * Calculate lateral wheel slip
     * Used for drift angle calculation
     */
    float CalculateLateralSlip(float steeringAngle, float speed, float yawRate) const;
    
    /**
     * Calculate forward wheel slip
     * Used for traction calculation
     */
    float CalculateForwardSlip(float throttle, float brake, float speed) const;
    
    /**
     * Get lateral slip
     */
    float GetLateralSlip() const { return m_lateralSlip; }
    
    /**
     * Get forward slip
     */
    float GetForwardSlip() const { return m_forwardSlip; }
    
    // ============================================
    // Update
    // ============================================
    
    /**
     * Update slip calculations
     */
    void Update(float deltaTime, float steering, float throttle, float brake, 
                float speed, float yawRate);
    
    // ============================================
    // Properties
    // ============================================
    
    void SetMaxLateralSlip(float maxSlip) { m_maxLateralSlip = maxSlip; }
    float GetMaxLateralSlip() const { return m_maxLateralSlip; }
    
    void SetMaxForwardSlip(float maxSlip) { m_maxForwardSlip = maxSlip; }
    float GetMaxForwardSlip() const { return m_maxForwardSlip; }

private:
    // Current slip values
    float m_lateralSlip = 0.0f;
    float m_forwardSlip = 0.0f;
    
    // Limits
    float m_maxLateralSlip = 1.0f;
    float m_maxForwardSlip = 1.0f;
    
    // Constants (from binary)
    static constexpr float LATERAL_SLIP_FACTOR = 0.6f;  // From WheelSlipLateral
    static constexpr float FORWARD_SLIP_FACTOR = 0.4f;  // From WheelSlipForward
};

/**
 * DriftController - Controller for drift mechanics
 * 
 * Combines DriftDriver and WheelSlipController
 */
class DriftController {
public:
    DriftController();
    ~DriftController();
    
    // ============================================
    // Initialization
    // ============================================
    
    bool Initialize();
    void Shutdown();
    
    // ============================================
    // Control
    // ============================================
    
    /**
     * Update drift controller
     */
    void Update(float deltaTime);
    
    /**
     * Apply input to drift system
     */
    void ApplyInput(float throttle, float brake, float steering, float handbrake);
    
    // ============================================
    // Access
    // ============================================
    
    DriftDriver* GetDriftDriver() { return &m_driftDriver; }
    WheelSlipController* GetSlipController() { return &m_slipController; }
    
    // ============================================
    // State
    // ============================================
    
    bool IsDrifting() const { return m_driftDriver.IsDrifting(); }
    float GetDriftAngle() const { return m_driftDriver.GetDriftAngle(); }
    float GetDriftScore() const { return m_driftDriver.GetDriftScore(); }

private:
    DriftDriver m_driftDriver;
    WheelSlipController m_slipController;
    bool m_initialized = false;
};

// ============================================
// Implementation
// ============================================

inline DriftDriver::DriftDriver() {
    m_state = DriftState::None;
    m_driftAngle = 0.0f;
    m_slipAngle = 0.0f;
    m_yawRate = 0.0f;
    m_driftScore = 0.0f;
    m_currentSpeed = 0.0f;
    m_previousSpeed = 0.0f;
    ZeroMemory(&m_lastSnapshot, sizeof(m_lastSnapshot));
}

inline DriftDriver::~DriftDriver() {
    Shutdown();
}

inline void DriftDriver::Initialize() {
    m_state = DriftState::None;
    m_driftAngle = 0.0f;
    m_slipAngle = 0.0f;
    m_yawRate = 0.0f;
    m_driftScore = 0.0f;
}

inline void DriftDriver::Shutdown() {
    m_vehicle = nullptr;
}

inline void DriftDriver::Update(float deltaTime) {
    if (!m_vehicle) return;
    
    // Update drift state
    UpdateDriftState(deltaTime);
    
    // Update drift angle
    UpdateDriftAngle(deltaTime);
    
    // Update yaw rate
    UpdateYawRate(deltaTime);
    
    // Calculate drift score
    if (IsDrifting()) {
        CalculateDriftScore(deltaTime);
    }
}

inline void DriftDriver::ApplyDriftInput(float throttle, float brake, float steering, float handbrake) {
    m_throttle = std::max(0.0f, std::min(1.0f, throttle));
    m_brake = std::max(0.0f, std::min(1.0f, brake));
    m_steering = std::max(-1.0f, std::min(1.0f, steering));
    m_handbrake = std::max(0.0f, std::min(1.0f, handbrake));
}

inline float DriftDriver::CalculateDriftAngle() const {
    // Drift angle is the angle between velocity and heading
    // Simplified calculation
    return m_driftAngle;
}

inline void DriftDriver::UpdateDriftState(float deltaTime) {
    bool wasDrifting = IsDrifting();
    bool canDrift = CheckDriftConditions();
    
    if (canDrift) {
        if (std::abs(m_driftAngle) > DRIFT_ANGLE_THRESHOLD) {
            if (m_state == DriftState::None || m_state == DriftState::Initiating) {
                m_state = DriftState::Sustaining;
            }
        } else if (m_state == DriftState::None) {
            m_state = DriftState::Initiating;
        }
    } else {
        if (wasDrifting) {
            m_state = DriftState::Exiting;
        } else {
            m_state = DriftState::None;
        }
    }
    
    // Check for spin
    if (std::abs(m_yawRate) > 5.0f) {
        m_state = DriftState::Spinning;
    }
}

inline void DriftDriver::UpdateDriftAngle(float deltaTime) {
    // Calculate drift angle based on steering, speed, and yaw
    float steeringFactor = m_steering * 0.5f;
    float speedFactor = m_currentSpeed / 100.0f;  // Normalize speed
    float yawFactor = m_yawRate * 0.1f;
    
    // Combine factors
    float targetAngle = steeringFactor * speedFactor + yawFactor;
    
    // Smooth interpolation
    float lerpSpeed = 5.0f * deltaTime;
    m_driftAngle = m_driftAngle * (1.0f - lerpSpeed) + targetAngle * lerpSpeed;
    
    // Clamp to max angle
    m_driftAngle = std::max(-MAX_DRIFT_ANGLE, std::min(MAX_DRIFT_ANGLE, m_driftAngle));
}

inline void DriftDriver::UpdateYawRate(float deltaTime) {
    // Yaw rate based on steering and speed
    float steeringInput = m_steering;
    float speedFactor = m_currentSpeed / 100.0f;
    
    // Calculate target yaw rate
    float targetYawRate = steeringInput * speedFactor * 2.0f;
    
    // Apply damping
    m_yawRate = m_yawRate * YAW_DAMPING + targetYawRate * (1.0f - YAW_DAMPING);
    
    // Handbrake increases yaw rate
    if (m_handbrake > 0.0f) {
        m_yawRate += m_handbrake * 3.0f * deltaTime;
    }
}

inline void DriftDriver::CalculateDriftScore(float deltaTime) {
    if (!IsDrifting()) return;
    
    // Score based on drift angle and speed
    float angleScore = std::abs(m_driftAngle) * 10.0f;
    float speedScore = m_currentSpeed * 0.1f;
    float yawScore = std::abs(m_yawRate) * 2.0f;
    
    float frameScore = (angleScore + speedScore + yawScore) * deltaTime;
    m_driftScore += frameScore;
}

inline bool DriftDriver::CheckDriftConditions() const {
    // Need minimum speed to drift
    if (m_currentSpeed < MIN_DRIFT_SPEED) {
        return false;
    }
    
    // Need steering input or handbrake
    if (std::abs(m_steering) < 0.1f && m_handbrake < 0.1f) {
        return false;
    }
    
    return true;
}

inline void DriftDriver::DoSnapshot() {
    // Take snapshot of current drift state
    m_lastSnapshot.driftAngle = m_driftAngle;
    m_lastSnapshot.speed = m_currentSpeed;
    m_lastSnapshot.throttle = m_throttle;
    m_lastSnapshot.steering = m_steering;
    m_lastSnapshot.timestamp = GetTickCount() / 1000.0f;
}

// WheelSlipController implementation
inline WheelSlipController::WheelSlipController() {
    m_lateralSlip = 0.0f;
    m_forwardSlip = 0.0f;
    m_maxLateralSlip = 1.0f;
    m_maxForwardSlip = 1.0f;
}

inline WheelSlipController::~WheelSlipController() {
}

inline float WheelSlipController::CalculateLateralSlip(float steeringAngle, float speed, float yawRate) const {
    // Lateral slip based on steering and yaw
    float steeringSlip = std::abs(steeringAngle) * LATERAL_SLIP_FACTOR;
    float yawSlip = std::abs(yawRate) * 0.1f;
    float speedFactor = speed / 100.0f;
    
    float slip = (steeringSlip + yawSlip) * speedFactor;
    return std::min(slip, m_maxLateralSlip);
}

inline float WheelSlipController::CalculateForwardSlip(float throttle, float brake, float speed) const {
    // Forward slip based on throttle/brake and speed
    float inputSlip = std::abs(throttle - brake) * FORWARD_SLIP_FACTOR;
    float speedFactor = speed / 100.0f;
    
    float slip = inputSlip * speedFactor;
    return std::min(slip, m_maxForwardSlip);
}

inline void WheelSlipController::Update(float deltaTime, float steering, float throttle, 
                                       float brake, float speed, float yawRate) {
    // Update lateral slip
    m_lateralSlip = CalculateLateralSlip(steering, speed, yawRate);
    
    // Update forward slip
    m_forwardSlip = CalculateForwardSlip(throttle, brake, speed);
}

// DriftController implementation
inline DriftController::DriftController() {
    m_initialized = false;
}

inline DriftController::~DriftController() {
    Shutdown();
}

inline bool DriftController::Initialize() {
    if (m_initialized) {
        return true;
    }
    
    m_driftDriver.Initialize();
    m_initialized = true;
    return true;
}

inline void DriftController::Shutdown() {
    m_driftDriver.Shutdown();
    m_initialized = false;
}

inline void DriftController::Update(float deltaTime) {
    if (!m_initialized) return;
    
    // Update drift driver
    m_driftDriver.Update(deltaTime);
    
    // Update slip controller with current state
    float steering = m_driftDriver.m_steering;
    float throttle = m_driftDriver.m_throttle;
    float brake = m_driftDriver.m_brake;
    float speed = m_driftDriver.m_currentSpeed;
    float yawRate = m_driftDriver.GetYawRate();
    
    m_slipController.Update(deltaTime, steering, throttle, brake, speed, yawRate);
}

inline void DriftController::ApplyInput(float throttle, float brake, float steering, float handbrake) {
    m_driftDriver.ApplyDriftInput(throttle, brake, steering, handbrake);
}

