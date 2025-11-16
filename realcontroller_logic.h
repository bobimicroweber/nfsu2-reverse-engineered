// RealController Class Logic
// Extracted from SPEED2.EXE disassembly
// RealController located at offset 0x385298

#pragma once

#include "carcontroller_enhanced.h"
#include <cmath>
#include <algorithm>

namespace RealControllerLogic {

// ============================================
// Constants Extracted from Binary
// ============================================

// Floating point constants near RealController:
const float REALCTRL_CONST_384AC8 = 0.000000f; // 0x384AC8
const float REALCTRL_CONST_384ACC = 0.000000f; // 0x384ACC
const float REALCTRL_CONST_384AD0 = 0.000000f; // 0x384AD0
const float REALCTRL_CONST_384AD4 = 0.000000f; // 0x384AD4
const float REALCTRL_CONST_384AD8 = 0.000000f; // 0x384AD8
const float REALCTRL_CONST_384ADC = 0.000000f; // 0x384ADC
const float REALCTRL_CONST_384AE0 = 0.000000f; // 0x384AE0
const float REALCTRL_CONST_384AE4 = 0.000000f; // 0x384AE4
const float REALCTRL_CONST_384AE8 = 0.000000f; // 0x384AE8
const float REALCTRL_CONST_384AEC = 0.000000f; // 0x384AEC
const float REALCTRL_CONST_384AF0 = 0.000000f; // 0x384AF0
const float REALCTRL_CONST_384AF4 = -0.045455f; // 0x384AF4
const float REALCTRL_CONST_384AF8 = 0.000000f; // 0x384AF8
const float REALCTRL_CONST_384AFC = 0.000000f; // 0x384AFC
const float REALCTRL_CONST_384B00 = 0.000000f; // 0x384B00

// ============================================
// RealController VTable Methods
// ============================================

// VTable at offset 0x384768
// VTable[0] -> 0x0040DB20
// VTable[1] -> 0x0040DAC0
// VTable[2] -> 0x0040CA40
// VTable[3] -> 0x0040CA50
// VTable[4] -> 0x0040C9C0

// ============================================
// RealController - Realistic Physics Control
// ============================================

/**
 * RealController class
 * Provides realistic physics-based vehicle control
 * More accurate than basic CarController
 */
class RealController {
public:
    RealController();
    virtual ~RealController();
    
    // ============================================
    // Realistic Steering Control
    // ============================================
    
    /**
     * Apply realistic steering with physics simulation
     * Takes into account vehicle speed, weight, and traction
     */
    void ApplyRealisticSteering(float input, float currentSpeed, float deltaTime) {
        // Input processing
        input = std::max(-1.0f, std::min(1.0f, input));
        
        // Realistic steering: speed affects steering response
        // At high speeds, steering is less responsive
        float speedFactor = 1.0f - (currentSpeed / 250.0f) * 0.5f;
        if (speedFactor < 0.3f) speedFactor = 0.3f;
        
        // Apply steering with speed-dependent sensitivity
        float steeringDelta = input * 0.000250f * speedFactor * deltaTime;
        
        // Update steering angle
        float currentAngle = m_steeringAngle;
        float newAngle = currentAngle + steeringDelta;
        
        // Clamp to max angle
        newAngle = std::max(-2.237f, std::min(2.237f, newAngle));
        m_steeringAngle = newAngle;
        
        // Realistic steering affects speed more at higher angles
        float speedReduction = std::abs(newAngle) * 0.15f * (currentSpeed / 250.0f);
        m_effectiveSpeed = currentSpeed * (1.0f - speedReduction);
    }
    
    // ============================================
    // Realistic Acceleration/Deceleration
    // ============================================
    
    /**
     * Apply realistic acceleration
     * Accounts for engine power, weight, and traction
     */
    void ApplyRealisticAcceleration(float throttle, float deltaTime) {
        throttle = std::max(0.0f, std::min(1.0f, throttle));
        
        float currentSpeed = m_currentSpeed;
        float maxSpeed = 250.0f;
        
        if (currentSpeed < maxSpeed && throttle > 0.0f) {
            // Realistic acceleration: decreases as speed increases
            float speedRatio = currentSpeed / maxSpeed;
            float accelerationFactor = 0.447027f * (1.0f - speedRatio * 0.5f);
            
            float acceleration = throttle * accelerationFactor * deltaTime;
            float newSpeed = currentSpeed + acceleration;
            
            if (newSpeed > maxSpeed) newSpeed = maxSpeed;
            m_currentSpeed = newSpeed;
        }
    }
    
    /**
     * Apply realistic braking
     * Accounts for weight transfer and traction
     */
    void ApplyRealisticBraking(float brake, float deltaTime) {
        brake = std::max(0.0f, std::min(1.0f, brake));
        
        float currentSpeed = m_currentSpeed;
        
        if (currentSpeed > 0.0f && brake > 0.0f) {
            // Realistic braking: more effective at higher speeds initially
            float speedFactor = 1.0f + (currentSpeed / 250.0f) * 0.3f;
            float deceleration = brake * 0.447027f * speedFactor * deltaTime;
            
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
        // Apply natural friction (more realistic)
        float friction = 0.330000f;
        
        // Friction increases with speed
        float speedDependentFriction = friction * (1.0f + (m_currentSpeed / 250.0f) * 0.2f);
        
        if (m_throttle == 0.0f && m_brake == 0.0f) {
            float frictionDecel = m_currentSpeed * speedDependentFriction * deltaTime;
            m_currentSpeed -= frictionDecel;
            if (m_currentSpeed < 0.0f) m_currentSpeed = 0.0f;
        }
        
        // Realistic steering decay (steering returns to center)
        if (std::abs(m_steeringAngle) > 0.01f) {
            float decayRate = 2.0f * deltaTime;
            if (m_steeringAngle > 0.0f) {
                m_steeringAngle -= decayRate;
                if (m_steeringAngle < 0.0f) m_steeringAngle = 0.0f;
            } else {
                m_steeringAngle += decayRate;
                if (m_steeringAngle > 0.0f) m_steeringAngle = 0.0f;
            }
        }
        
        // Calculate realistic wheel slip
        float lateralSlip = std::abs(m_steeringAngle) * 0.6f * (m_currentSpeed / 250.0f);
        float forwardSlip = std::abs(m_throttle - m_brake) * 0.4f * (m_currentSpeed / 250.0f);
        
        m_wheelSlipLateral = lateralSlip;
        m_wheelSlipForward = forwardSlip;
    }
    
    // ============================================
    // Getters/Setters
    // ============================================
    
    float GetSteeringAngle() const {{ return m_steeringAngle; }}
    float GetCurrentSpeed() const {{ return m_currentSpeed; }}
    float GetEffectiveSpeed() const {{ return m_effectiveSpeed; }}
    float GetWheelSlipLateral() const {{ return m_wheelSlipLateral; }}
    float GetWheelSlipForward() const {{ return m_wheelSlipForward; }}
    
    void SetThrottle(float throttle) {{ m_throttle = throttle; }}
    void SetBrake(float brake) {{ m_brake = brake; }}
    
private:
    // State variables
    float m_steeringAngle = 0.0f;
    float m_currentSpeed = 0.0f;
    float m_effectiveSpeed = 0.0f;
    float m_throttle = 0.0f;
    float m_brake = 0.0f;
    
    // Physics
    float m_wheelSlipLateral = 0.0f;
    float m_wheelSlipForward = 0.0f;
    
    // Constants from binary
    static constexpr float STEERING_SENSITIVITY = 0.000250f;
    static constexpr float MAX_STEERING_ANGLE = 2.237000f;
    static constexpr float MAX_SPEED = 250.000000f;
    static constexpr float FRICTION_COEFFICIENT = 0.330000f;
    static constexpr float ACCELERATION_FACTOR = 0.447027f;
}};

}} // namespace RealControllerLogic

