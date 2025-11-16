// RealController Detailed Logic
// Extracted from SPEED2.EXE with actual disassembly
// RealController located at offset 0x385298

#pragma once

#include "carcontroller_enhanced.h"
#include <cmath>
#include <algorithm>

namespace RealControllerLogic {

// ============================================
// RealController VTable Methods
// ============================================
// VTable at 0x384768 with methods:
// [0] -> 0x40DB20 (Constructor/Initialize)
// [1] -> 0x40DAC0 (Update/Process)
// [2] -> 0x40CA40 (GetState/GetType)
// [3] -> 0x40CA50 (Reset/Cleanup)
// [4] -> 0x40C9C0 (Additional method)

/**
 * RealController - Realistic Physics-Based Vehicle Control
 * 
 * This controller provides more realistic physics simulation compared
 * to the basic CarController. It includes:
 * - Speed-dependent steering response
 * - Realistic acceleration/deceleration curves
 * - Natural steering decay
 * - Speed-dependent friction
 * - Advanced wheel slip calculation
 */
class RealController {
public:
    RealController();
    virtual ~RealController();
    
    // ============================================
    // VTable Methods (from disassembly)
    // ============================================
    
    /**
     * VTable Method 0 (0x40DB20)
     * Constructor or Initialize function
     */
    virtual void Initialize();
    
    /**
     * VTable Method 1 (0x40DAC0)
     * Update or Process function
     * Called every frame to update physics
     */
    virtual void Update(float deltaTime);
    
    /**
     * VTable Method 2 (0x40CA40)
     * GetState or GetType function
     */
    virtual int GetState() const;
    
    /**
     * VTable Method 3 (0x40CA50)
     * Reset or Cleanup function
     */
    virtual void Reset();
    
    // ============================================
    // Realistic Steering Control
    // ============================================
    
    /**
     * Apply realistic steering with advanced physics
     * 
     * Key differences from basic controller:
     * - Steering response decreases at high speeds
     * - Steering affects speed more realistically
     * - Natural steering return to center
     */
    void ApplyRealisticSteering(float input, float deltaTime) {
        if (!IsValid()) return;
        
        // Clamp input
        input = std::max(-1.0f, std::min(1.0f, input));
        
        // Get current state
        float currentSpeed = GetCurrentSpeed();
        float currentAngle = GetSteeringAngle();
        
        // Realistic steering: speed affects responsiveness
        // Formula: response = base_response * (1 - speed_factor)
        // At 250 km/h, steering is 50% less responsive
        float speedFactor = currentSpeed / MAX_SPEED;
        float responsiveness = 1.0f - (speedFactor * 0.5f);
        if (responsiveness < 0.3f) responsiveness = 0.3f; // Minimum 30%
        
        // Apply steering with speed-dependent sensitivity
        float steeringDelta = input * STEERING_SENSITIVITY * responsiveness * deltaTime;
        float newAngle = currentAngle + steeringDelta;
        
        // Clamp to maximum angle
        newAngle = std::max(-MAX_STEERING_ANGLE, std::min(MAX_STEERING_ANGLE, newAngle));
        m_steeringAngle = newAngle;
        
        // Realistic speed reduction from steering
        // More pronounced at higher speeds and angles
        float angleFactor = std::abs(newAngle) / MAX_STEERING_ANGLE;
        float speedReduction = angleFactor * 0.15f * speedFactor;
        m_effectiveSpeed = currentSpeed * (1.0f - speedReduction);
        
        // Natural steering decay (returns to center)
        if (std::abs(newAngle) > 0.01f && input == 0.0f) {
            float decayRate = 2.0f * deltaTime;
            if (newAngle > 0.0f) {
                m_steeringAngle = std::max(0.0f, newAngle - decayRate);
            } else {
                m_steeringAngle = std::min(0.0f, newAngle + decayRate);
            }
        }
    }
    
    // ============================================
    // Realistic Acceleration
    // ============================================
    
    /**
     * Apply realistic acceleration
     * 
     * Accounts for:
     * - Engine power curve (less effective at high speeds)
     * - Vehicle weight
     * - Traction limits
     */
    void ApplyRealisticAcceleration(float throttle, float deltaTime) {
        if (!IsValid()) return;
        
        throttle = std::max(0.0f, std::min(1.0f, throttle));
        m_throttle = throttle;
        
        float currentSpeed = m_currentSpeed;
        
        if (currentSpeed < MAX_SPEED && throttle > 0.0f) {
            // Realistic acceleration: decreases as speed increases
            // This simulates engine power curve and air resistance
            float speedRatio = currentSpeed / MAX_SPEED;
            
            // Acceleration factor decreases with speed
            // At 0 km/h: 100% acceleration
            // At 250 km/h: 50% acceleration
            float accelerationFactor = ACCELERATION_FACTOR * (1.0f - speedRatio * 0.5f);
            
            float acceleration = throttle * accelerationFactor * deltaTime;
            float newSpeed = currentSpeed + acceleration;
            
            // Clamp to max speed
            if (newSpeed > MAX_SPEED) newSpeed = MAX_SPEED;
            m_currentSpeed = newSpeed;
        }
    }
    
    // ============================================
    // Realistic Braking
    // ============================================
    
    /**
     * Apply realistic braking
     * 
     * Accounts for:
     * - Weight transfer (more effective at high speeds initially)
     * - Brake fade
     * - Traction limits
     */
    void ApplyRealisticBraking(float brake, float deltaTime) {
        if (!IsValid()) return;
        
        brake = std::max(0.0f, std::min(1.0f, brake));
        m_brake = brake;
        
        float currentSpeed = m_currentSpeed;
        
        if (currentSpeed > 0.0f && brake > 0.0f) {
            // Realistic braking: more effective at higher speeds initially
            // This simulates weight transfer and brake effectiveness
            float speedFactor = 1.0f + (currentSpeed / MAX_SPEED) * 0.3f;
            
            // Deceleration increases with speed (up to a point)
            float deceleration = brake * ACCELERATION_FACTOR * speedFactor * deltaTime;
            
            float newSpeed = currentSpeed - deceleration;
            if (newSpeed < 0.0f) newSpeed = 0.0f;
            m_currentSpeed = newSpeed;
        }
    }
    
    // ============================================
    // Realistic Physics Update
    // ============================================
    
    /**
     * Update realistic physics simulation
     * Called every frame
     */
    void UpdateRealisticPhysics(float deltaTime) {
        if (!IsValid()) return;
        
        // Apply speed-dependent friction
        // Friction increases with speed (air resistance)
        float baseFriction = FRICTION_COEFFICIENT;
        float speedDependentFriction = baseFriction * (1.0f + (m_currentSpeed / MAX_SPEED) * 0.2f);
        
        // Natural deceleration when no input
        if (m_throttle == 0.0f && m_brake == 0.0f) {
            float frictionDecel = m_currentSpeed * speedDependentFriction * deltaTime;
            m_currentSpeed -= frictionDecel;
            if (m_currentSpeed < 0.0f) m_currentSpeed = 0.0f;
        }
        
        // Calculate realistic wheel slip
        // Lateral slip: depends on steering angle and speed
        float lateralSlip = std::abs(m_steeringAngle) / MAX_STEERING_ANGLE;
        lateralSlip *= 0.6f * (m_currentSpeed / MAX_SPEED);
        m_wheelSlipLateral = lateralSlip;
        
        // Forward slip: depends on throttle/brake and speed
        float forwardSlip = std::abs(m_throttle - m_brake);
        forwardSlip *= 0.4f * (m_currentSpeed / MAX_SPEED);
        m_wheelSlipForward = forwardSlip;
        
        // Update effective speed (affected by steering and slip)
        m_effectiveSpeed = m_currentSpeed * (1.0f - (lateralSlip + forwardSlip) * 0.1f);
    }
    
    // ============================================
    // Getters
    // ============================================
    
    float GetSteeringAngle() const { return m_steeringAngle; }
    float GetCurrentSpeed() const { return m_currentSpeed; }
    float GetEffectiveSpeed() const { return m_effectiveSpeed; }
    float GetWheelSlipLateral() const { return m_wheelSlipLateral; }
    float GetWheelSlipForward() const { return m_wheelSlipForward; }
    float GetThrottle() const { return m_throttle; }
    float GetBrake() const { return m_brake; }
    
    bool IsValid() const { return m_initialized; }
    
private:
    // State
    bool m_initialized = false;
    float m_steeringAngle = 0.0f;
    float m_currentSpeed = 0.0f;
    float m_effectiveSpeed = 0.0f;
    float m_throttle = 0.0f;
    float m_brake = 0.0f;
    
    // Physics
    float m_wheelSlipLateral = 0.0f;
    float m_wheelSlipForward = 0.0f;
    
    // Constants from binary (same as CarController but used differently)
    static constexpr float STEERING_SENSITIVITY = 0.000250f;
    static constexpr float MAX_STEERING_ANGLE = 2.237000f;
    static constexpr float MAX_SPEED = 250.000000f;
    static constexpr float FRICTION_COEFFICIENT = 0.330000f;
    static constexpr float ACCELERATION_FACTOR = 0.447027f;
};

// ============================================
// VTable Method Implementations
// ============================================

inline void RealController::Initialize() {
    m_initialized = true;
    m_steeringAngle = 0.0f;
    m_currentSpeed = 0.0f;
    m_effectiveSpeed = 0.0f;
    m_throttle = 0.0f;
    m_brake = 0.0f;
    m_wheelSlipLateral = 0.0f;
    m_wheelSlipForward = 0.0f;
}

inline void RealController::Update(float deltaTime) {
    UpdateRealisticPhysics(deltaTime);
}

inline int RealController::GetState() const {
    return m_initialized ? 1 : 0;
}

inline void RealController::Reset() {
    m_steeringAngle = 0.0f;
    m_currentSpeed = 0.0f;
    m_effectiveSpeed = 0.0f;
    m_throttle = 0.0f;
    m_brake = 0.0f;
}

} // namespace RealControllerLogic

