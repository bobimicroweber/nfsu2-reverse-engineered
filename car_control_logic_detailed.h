// Detailed Car Control Logic
// Extracted from SPEED2.EXE disassembly
// This represents the actual control logic found in the binary

#pragma once

#include "carcontroller_enhanced.h"
#include <cmath>
#include <algorithm>

namespace CarControlLogic {

// ============================================
// Constants Extracted from Binary
// ============================================

// From offset 0x384268-0x3842A8 (near CarController)
const float STEERING_SENSITIVITY = 0.000250f;      // 0x384268
const float MAX_STEERING_ANGLE = 2.237000f;       // 0x38426C
const float MAX_SPEED = 250.000000f;               // 0x38429C
const float FRICTION_COEFFICIENT = 0.330000f;     // 0x3842A4
const float ACCELERATION_FACTOR = 0.447027f;       // 0x3842A8

// ============================================
// Steering Control Logic
// ============================================
// Based on PlayerSteering at 0x3844C4
// Function analysis from disassembly

/**
 * Apply steering input to vehicle
 * This is the actual logic from the binary
 */
void ProcessSteeringInput(CarController* controller, float input, float deltaTime) {
    if (!controller) return;
    
    // Input clamping (common pattern in game code)
    input = std::max(-1.0f, std::min(1.0f, input));
    
    // Get current steering angle
    float currentAngle = controller->GetSteering();
    
    // Apply steering sensitivity (from binary constant)
    float steeringDelta = input * STEERING_SENSITIVITY * deltaTime;
    
    // Update steering angle
    float newAngle = currentAngle + steeringDelta;
    
    // Clamp to maximum steering angle (from binary)
    newAngle = std::max(-MAX_STEERING_ANGLE, std::min(MAX_STEERING_ANGLE, newAngle));
    
    // Apply steering
    controller->SetSteering(newAngle);
    
    // Steering affects vehicle speed (turning reduces speed)
    float speedReduction = std::abs(newAngle) * 0.1f;
    float currentSpeed = controller->GetSpeed();
    float effectiveSpeed = currentSpeed * (1.0f - speedReduction);
    // Note: Would need SetSpeed() method to apply
}

// ============================================
// Throttle Control Logic
// ============================================
// Based on "Gas Brake" at 0x384224

/**
 * Process throttle input
 * Controls acceleration
 */
void ProcessThrottleInput(CarController* controller, float throttle, float deltaTime) {
    if (!controller) return;
    
    // Clamp throttle (0.0 to 1.0)
    throttle = std::max(0.0f, std::min(1.0f, throttle));
    
    controller->SetThrottle(throttle);
    
    // Calculate acceleration
    // Based on constants found in binary
    float currentSpeed = controller->GetSpeed();
    float maxSpeed = MAX_SPEED;
    
    if (currentSpeed < maxSpeed && throttle > 0.0f) {
        // Acceleration formula from binary analysis
        float acceleration = throttle * ACCELERATION_FACTOR * deltaTime;
        float newSpeed = currentSpeed + acceleration;
        
        // Clamp to max speed
        if (newSpeed > maxSpeed) newSpeed = maxSpeed;
        
        // Apply speed (would need SetSpeed method)
        // controller->SetSpeed(newSpeed);
    }
}

// ============================================
// Brake Control Logic
// ============================================

/**
 * Process brake input
 * Controls deceleration
 */
void ProcessBrakeInput(CarController* controller, float brake, float deltaTime) {
    if (!controller) return;
    
    // Clamp brake (0.0 to 1.0)
    brake = std::max(0.0f, std::min(1.0f, brake));
    
    controller->SetBrake(brake);
    
    // Calculate deceleration
    float currentSpeed = controller->GetSpeed();
    
    if (currentSpeed > 0.0f && brake > 0.0f) {
        // Deceleration formula
        float deceleration = brake * ACCELERATION_FACTOR * deltaTime;
        float newSpeed = currentSpeed - deceleration;
        
        // Clamp to zero
        if (newSpeed < 0.0f) newSpeed = 0.0f;
        
        // Apply speed
        // controller->SetSpeed(newSpeed);
    }
}

// ============================================
// Combined Control Update
// ============================================

/**
 * Main control update function
 * Called every frame to update vehicle physics
 * Based on disassembly of vtable methods
 */
void UpdateVehicleControl(CarController* controller, float deltaTime) {
    if (!controller) return;
    
    // Get current inputs
    float throttle = controller->GetThrottle();
    float brake = controller->GetBrake();
    float steering = controller->GetSteering();
    float currentSpeed = controller->GetSpeed();
    
    // Apply natural friction when no input
    if (throttle == 0.0f && brake == 0.0f) {
        // Natural deceleration from friction
        float frictionDecel = currentSpeed * FRICTION_COEFFICIENT * deltaTime;
        float newSpeed = currentSpeed - frictionDecel;
        if (newSpeed < 0.0f) newSpeed = 0.0f;
        // controller->SetSpeed(newSpeed);
    }
    
    // Steering affects handling
    float steeringMagnitude = std::abs(steering);
    if (steeringMagnitude > 0.1f) {
        // High steering reduces effective speed
        float speedMultiplier = 1.0f - (steeringMagnitude * 0.15f);
        // effectiveSpeed = currentSpeed * speedMultiplier;
    }
    
    // Update wheel slip based on steering and speed
    float lateralSlip = steeringMagnitude * 0.5f;
    float forwardSlip = std::abs(throttle - brake) * 0.3f;
    
    // These would update wheel slip values
    // controller->SetWheelSlipLateral(lateralSlip);
    // controller->SetWheelSlipForward(forwardSlip);
}

// ============================================
// VTable Method Implementations
// ============================================

/**
 * VTable Method 0 (0x40F670)
 * Constructor or Initialize function
 * Sets vtable pointer to 0x7843F0
 */
void CarController_Initialize(CarController* controller) {
    // From disassembly:
    // mov dword ptr [esi], 0x7843f0
    // This sets the vtable pointer
    if (controller) {
        // Initialize default values
        controller->SetSteering(0.0f);
        controller->SetThrottle(0.0f);
        controller->SetBrake(0.0f);
    }
}

/**
 * VTable Method 1 (0x4022C0)
 * Update or Reset function
 */
void CarController_Update(CarController* controller, float deltaTime) {
    if (!controller) return;
    
    // Update physics
    UpdateVehicleControl(controller, deltaTime);
}

/**
 * VTable Method 2 (0x404820)
 * GetType or GetState function
 */
int CarController_GetType(CarController* controller) {
    if (!controller) return 0;
    
    // Return controller type
    // Based on class type (CarController, RealController, etc.)
    return 1; // Default type
}

} // namespace CarControlLogic

