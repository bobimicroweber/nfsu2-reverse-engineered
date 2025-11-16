// DriftDriver Class - Detailed Implementation
// Extracted from SPEED2.EXE
// DriftDriver located at offset 0x3844A4

#pragma once

#include <windows.h>
#include <cmath>
#include <algorithm>
#include <vector>
#include "carcontroller_enhanced.h"

// Forward declarations
class Vehicle;
class WheelSlipController;

/**
 * DriftDriver - Advanced drift driving controller
 * 
 * Found in binary at:
 * - 0x3844A4 (first occurrence)
 * - 0x385B2C (second occurrence)
 * 
 * Method found:
 * - DoSnapshot() at 0x3844AC
 * 
 * Provides advanced drift mechanics including:
 * - Drift angle calculation
 * - Yaw rate control
 * - Drift scoring
 * - Snapshot system
 * - State management
 */
class DriftDriver {
public:
    DriftDriver();
    ~DriftDriver();
    
    // ============================================
    // Initialization
    // ============================================
    
    /**
     * Initialize drift driver
     */
    void Initialize();
    
    /**
     * Shutdown drift driver
     */
    void Shutdown();
    
    // ============================================
    // Main Update
    // ============================================
    
    /**
     * Update drift driver
     * Called every frame
     */
    void Update(float deltaTime);
    
    // ============================================
    // Drift Control
    // ============================================
    
    /**
     * Apply drift input
     * @param throttle Throttle input (0.0 to 1.0)
     * @param brake Brake input (0.0 to 1.0)
     * @param steering Steering input (-1.0 to 1.0)
     * @param handbrake Handbrake input (0.0 to 1.0)
     */
    void ApplyInput(float throttle, float brake, float steering, float handbrake);
    
    /**
     * Calculate current drift angle
     * Returns angle in radians
     */
    float CalculateDriftAngle() const;
    
    /**
     * Get current drift angle
     */
    float GetDriftAngle() const { return m_driftAngle; }
    
    /**
     * Get slip angle
     */
    float GetSlipAngle() const { return m_slipAngle; }
    
    /**
     * Get yaw rate
     */
    float GetYawRate() const { return m_yawRate; }
    
    // ============================================
    // Drift Scoring
    // ============================================
    
    /**
     * Get current drift score
     */
    float GetDriftScore() const { return m_driftScore; }
    
    /**
     * Get total drift score (accumulated)
     */
    float GetTotalDriftScore() const { return m_totalDriftScore; }
    
    /**
     * Reset drift score
     */
    void ResetDriftScore();
    
    /**
     * Calculate drift score for current frame
     */
    float CalculateDriftScore(float deltaTime);
    
    // ============================================
    // Drift State
    // ============================================
    
    enum class DriftState {
        None,           // Not drifting
        Initiating,     // Starting drift
        Sustaining,     // Maintaining drift
        Exiting,        // Ending drift
        Spinning        // Out of control
    };
    
    DriftState GetState() const { return m_state; }
    bool IsDrifting() const { return m_state != DriftState::None; }
    bool IsSustaining() const { return m_state == DriftState::Sustaining; }
    
    // ============================================
    // Snapshot System
    // ============================================
    
    /**
     * DoSnapshot - Take a snapshot of drift state
     * 
     * Found in binary as "DriftDriver::DoSnapshot" at 0x3844AC
     * Captures current drift state for replay/debugging
     */
    void DoSnapshot();
    
    /**
     * Snapshot data structure
     */
    struct Snapshot {
        float driftAngle;        // Current drift angle
        float slipAngle;         // Slip angle
        float yawRate;           // Yaw rotation rate
        float speed;             // Current speed
        float throttle;          // Throttle input
        float brake;             // Brake input
        float steering;          // Steering input
        float handbrake;         // Handbrake input
        float driftScore;        // Current drift score
        DriftState state;        // Current drift state
        float timestamp;         // Timestamp
    };
    
    /**
     * Get last snapshot
     */
    const Snapshot& GetLastSnapshot() const { return m_lastSnapshot; }
    
    /**
     * Get snapshot history
     */
    const std::vector<Snapshot>& GetSnapshotHistory() const { return m_snapshotHistory; }
    
    /**
     * Clear snapshot history
     */
    void ClearSnapshotHistory() { m_snapshotHistory.clear(); }
    
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
    
    /**
     * Set wheel slip controller
     */
    void SetWheelSlipController(WheelSlipController* controller) {
        m_slipController = controller;
    }
    
    // ============================================
    // Drift Parameters
    // ============================================
    
    /**
     * Set minimum speed for drifting
     */
    void SetMinDriftSpeed(float speed) { m_minDriftSpeed = speed; }
    float GetMinDriftSpeed() const { return m_minDriftSpeed; }
    
    /**
     * Set maximum drift angle
     */
    void SetMaxDriftAngle(float angle) { m_maxDriftAngle = angle; }
    float GetMaxDriftAngle() const { return m_maxDriftAngle; }
    
    /**
     * Set drift angle threshold
     */
    void SetDriftAngleThreshold(float threshold) { m_driftAngleThreshold = threshold; }
    float GetDriftAngleThreshold() const { return m_driftAngleThreshold; }

private:
    // Vehicle reference
    Vehicle* m_vehicle = nullptr;
    WheelSlipController* m_slipController = nullptr;
    
    // Drift state
    DriftState m_state = DriftState::None;
    
    // Drift metrics
    float m_driftAngle = 0.0f;          // Current drift angle (radians)
    float m_slipAngle = 0.0f;            // Slip angle (radians)
    float m_yawRate = 0.0f;             // Yaw rotation rate (rad/s)
    float m_previousYawRate = 0.0f;     // Previous yaw rate
    
    // Drift scoring
    float m_driftScore = 0.0f;           // Current drift score
    float m_totalDriftScore = 0.0f;     // Total accumulated score
    float m_driftMultiplier = 1.0f;      // Score multiplier
    
    // Inputs
    float m_throttle = 0.0f;
    float m_brake = 0.0f;
    float m_steering = 0.0f;
    float m_handbrake = 0.0f;
    
    // Physics
    float m_currentSpeed = 0.0f;
    float m_previousSpeed = 0.0f;
    float m_velocityX = 0.0f;
    float m_velocityY = 0.0f;
    float m_velocityZ = 0.0f;
    float m_heading = 0.0f;              // Vehicle heading (radians)
    
    // Snapshot
    Snapshot m_lastSnapshot;
    std::vector<Snapshot> m_snapshotHistory;
    int m_maxSnapshots = 100;
    
    // Timing
    float m_driftStartTime = 0.0f;
    float m_driftDuration = 0.0f;
    float m_totalTime = 0.0f;
    
    // Constants (from binary analysis)
    static constexpr float MIN_DRIFT_SPEED = 30.0f;      // Minimum speed to drift (km/h)
    static constexpr float MAX_DRIFT_ANGLE = 1.57f;      // ~90 degrees (radians)
    static constexpr float DRIFT_ANGLE_THRESHOLD = 0.3f; // Angle to consider drifting (radians)
    static constexpr float YAW_DAMPING = 0.85f;          // Yaw rate damping factor
    static constexpr float SLIP_MULTIPLIER = 0.6f;       // From WheelSlipLateral
    static constexpr float DRIFT_SCORE_BASE = 10.0f;     // Base score per second
    static constexpr float DRIFT_SCORE_ANGLE_MULT = 50.0f; // Score multiplier for angle
    static constexpr float DRIFT_SCORE_SPEED_MULT = 0.1f;  // Score multiplier for speed
    
    // Internal methods
    void UpdateDriftState(float deltaTime);
    void UpdateDriftAngle(float deltaTime);
    void UpdateSlipAngle(float deltaTime);
    void UpdateYawRate(float deltaTime);
    void UpdateDriftScore(float deltaTime);
    bool CheckDriftConditions() const;
    void UpdatePhysics(float deltaTime);
    float CalculateAngleBetweenVectors(float v1x, float v1y, float v2x, float v2y) const;
};

// ============================================
// DriftDriver Implementation
// ============================================

inline DriftDriver::DriftDriver() {
    m_vehicle = nullptr;
    m_slipController = nullptr;
    m_state = DriftState::None;
    m_driftAngle = 0.0f;
    m_slipAngle = 0.0f;
    m_yawRate = 0.0f;
    m_previousYawRate = 0.0f;
    m_driftScore = 0.0f;
    m_totalDriftScore = 0.0f;
    m_driftMultiplier = 1.0f;
    m_currentSpeed = 0.0f;
    m_previousSpeed = 0.0f;
    m_heading = 0.0f;
    m_driftStartTime = 0.0f;
    m_driftDuration = 0.0f;
    m_totalTime = 0.0f;
    m_minDriftSpeed = MIN_DRIFT_SPEED;
    m_maxDriftAngle = MAX_DRIFT_ANGLE;
    m_driftAngleThreshold = DRIFT_ANGLE_THRESHOLD;
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
    m_totalDriftScore = 0.0f;
    m_driftStartTime = 0.0f;
    m_driftDuration = 0.0f;
    m_totalTime = 0.0f;
    m_snapshotHistory.clear();
}

inline void DriftDriver::Shutdown() {
    m_vehicle = nullptr;
    m_slipController = nullptr;
    m_snapshotHistory.clear();
}

inline void DriftDriver::Update(float deltaTime) {
    if (!m_vehicle) return;
    
    m_totalTime += deltaTime;
    
    // Update physics
    UpdatePhysics(deltaTime);
    
    // Update drift state
    UpdateDriftState(deltaTime);
    
    // Update drift angle
    UpdateDriftAngle(deltaTime);
    
    // Update slip angle
    UpdateSlipAngle(deltaTime);
    
    // Update yaw rate
    UpdateYawRate(deltaTime);
    
    // Update drift score
    if (IsDrifting()) {
        UpdateDriftScore(deltaTime);
    }
}

inline void DriftDriver::ApplyInput(float throttle, float brake, float steering, float handbrake) {
    m_throttle = std::max(0.0f, std::min(1.0f, throttle));
    m_brake = std::max(0.0f, std::min(1.0f, brake));
    m_steering = std::max(-1.0f, std::min(1.0f, steering));
    m_handbrake = std::max(0.0f, std::min(1.0f, handbrake));
}

inline float DriftDriver::CalculateDriftAngle() const {
    // Drift angle is the angle between velocity vector and heading
    // Simplified calculation
    return m_driftAngle;
}

inline void DriftDriver::UpdateDriftState(float deltaTime) {
    bool wasDrifting = IsDrifting();
    bool canDrift = CheckDriftConditions();
    
    if (canDrift) {
        if (std::abs(m_driftAngle) > m_driftAngleThreshold) {
            if (m_state == DriftState::None || m_state == DriftState::Initiating) {
                m_state = DriftState::Sustaining;
                if (m_driftStartTime == 0.0f) {
                    m_driftStartTime = m_totalTime;
                }
            }
        } else if (m_state == DriftState::None) {
            m_state = DriftState::Initiating;
        }
    } else {
        if (wasDrifting) {
            m_state = DriftState::Exiting;
        } else {
            m_state = DriftState::None;
            m_driftStartTime = 0.0f;
            m_driftDuration = 0.0f;
        }
    }
    
    // Check for spin (excessive yaw rate)
    if (std::abs(m_yawRate) > 5.0f) {
        m_state = DriftState::Spinning;
    }
    
    // Update drift duration
    if (IsDrifting()) {
        m_driftDuration = m_totalTime - m_driftStartTime;
    }
}

inline void DriftDriver::UpdateDriftAngle(float deltaTime) {
    // Calculate drift angle based on steering, speed, and yaw
    float steeringFactor = m_steering * 0.5f;
    float speedFactor = m_currentSpeed / 100.0f;  // Normalize speed
    float yawFactor = m_yawRate * 0.1f;
    
    // Combine factors
    float targetAngle = steeringFactor * speedFactor + yawFactor;
    
    // Apply handbrake effect (increases drift angle)
    if (m_handbrake > 0.0f) {
        targetAngle += m_handbrake * 0.5f * speedFactor;
    }
    
    // Smooth interpolation
    float lerpSpeed = 5.0f * deltaTime;
    m_driftAngle = m_driftAngle * (1.0f - lerpSpeed) + targetAngle * lerpSpeed;
    
    // Clamp to max angle
    m_driftAngle = std::max(-m_maxDriftAngle, std::min(m_maxDriftAngle, m_driftAngle));
}

inline void DriftDriver::UpdateSlipAngle(float deltaTime) {
    // Slip angle is related to drift angle and wheel slip
    if (m_slipController) {
        float lateralSlip = m_slipController->GetLateralSlip();
        m_slipAngle = m_driftAngle * (1.0f + lateralSlip * SLIP_MULTIPLIER);
    } else {
        // Fallback calculation
        m_slipAngle = m_driftAngle * 1.2f;
    }
    
    // Clamp slip angle
    m_slipAngle = std::max(-m_maxDriftAngle * 1.5f, std::min(m_maxDriftAngle * 1.5f, m_slipAngle));
}

inline void DriftDriver::UpdateYawRate(float deltaTime) {
    // Yaw rate based on steering, speed, and drift angle
    float steeringInput = m_steering;
    float speedFactor = m_currentSpeed / 100.0f;
    
    // Calculate target yaw rate
    float targetYawRate = steeringInput * speedFactor * 2.0f;
    
    // Drift angle affects yaw rate
    targetYawRate += m_driftAngle * 1.5f;
    
    // Apply damping
    m_previousYawRate = m_yawRate;
    m_yawRate = m_yawRate * YAW_DAMPING + targetYawRate * (1.0f - YAW_DAMPING);
    
    // Handbrake increases yaw rate
    if (m_handbrake > 0.0f) {
        m_yawRate += m_handbrake * 3.0f * deltaTime;
    }
}

inline void DriftDriver::UpdateDriftScore(float deltaTime) {
    if (!IsDrifting()) return;
    
    // Score based on drift angle, speed, and duration
    float angleScore = std::abs(m_driftAngle) * DRIFT_SCORE_ANGLE_MULT;
    float speedScore = m_currentSpeed * DRIFT_SCORE_SPEED_MULT;
    float durationScore = m_driftDuration * DRIFT_SCORE_BASE;
    float yawScore = std::abs(m_yawRate) * 2.0f;
    
    // Combine scores
    float frameScore = (angleScore + speedScore + durationScore + yawScore) * deltaTime * m_driftMultiplier;
    
    m_driftScore = frameScore;
    m_totalDriftScore += frameScore;
}

inline bool DriftDriver::CheckDriftConditions() const {
    // Need minimum speed to drift
    if (m_currentSpeed < m_minDriftSpeed) {
        return false;
    }
    
    // Need steering input or handbrake
    if (std::abs(m_steering) < 0.1f && m_handbrake < 0.1f) {
        return false;
    }
    
    return true;
}

inline void DriftDriver::UpdatePhysics(float deltaTime) {
    // Update speed (would get from vehicle)
    // m_currentSpeed = vehicle->GetSpeed();
    
    // Update heading (would get from vehicle)
    // m_heading = vehicle->GetHeading();
    
    // Update velocity (would get from vehicle)
    // vehicle->GetVelocity(m_velocityX, m_velocityY, m_velocityZ);
}

inline float DriftDriver::CalculateAngleBetweenVectors(float v1x, float v1y, float v2x, float v2y) const {
    // Calculate angle between two 2D vectors
    float dot = v1x * v2x + v1y * v2y;
    float mag1 = std::sqrt(v1x * v1x + v1y * v1y);
    float mag2 = std::sqrt(v2x * v2x + v2y * v2y);
    
    if (mag1 < 0.001f || mag2 < 0.001f) return 0.0f;
    
    float cosAngle = dot / (mag1 * mag2);
    cosAngle = std::max(-1.0f, std::min(1.0f, cosAngle));
    
    return std::acos(cosAngle);
}

inline void DriftDriver::DoSnapshot() {
    // Take snapshot of current drift state
    // Found in binary as "DriftDriver::DoSnapshot" at 0x3844AC
    
    m_lastSnapshot.driftAngle = m_driftAngle;
    m_lastSnapshot.slipAngle = m_slipAngle;
    m_lastSnapshot.yawRate = m_yawRate;
    m_lastSnapshot.speed = m_currentSpeed;
    m_lastSnapshot.throttle = m_throttle;
    m_lastSnapshot.brake = m_brake;
    m_lastSnapshot.steering = m_steering;
    m_lastSnapshot.handbrake = m_handbrake;
    m_lastSnapshot.driftScore = m_driftScore;
    m_lastSnapshot.state = m_state;
    m_lastSnapshot.timestamp = m_totalTime;
    
    // Add to history
    m_snapshotHistory.push_back(m_lastSnapshot);
    
    // Limit history size
    if (m_snapshotHistory.size() > m_maxSnapshots) {
        m_snapshotHistory.erase(m_snapshotHistory.begin());
    }
}

inline void DriftDriver::ResetDriftScore() {
    m_driftScore = 0.0f;
    m_totalDriftScore = 0.0f;
    m_driftStartTime = 0.0f;
    m_driftDuration = 0.0f;
}

inline float DriftDriver::CalculateDriftScore(float deltaTime) {
    UpdateDriftScore(deltaTime);
    return m_driftScore;
}

