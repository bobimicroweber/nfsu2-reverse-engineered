// Car Control Logic Implementation
// Based on disassembled code from SPEED2.EXE

#include "car_control_logic.h"

namespace CarControlLogic {

// Detailed implementation based on binary analysis
// Constants extracted from SPEED2.EXE:

const float STEERING_SENSITIVITY = 0.000250f;
const float MAX_STEERING_ANGLE = 2.237000f;
const float MAX_SPEED = 250.0f;
const float FRICTION_COEFFICIENT = 0.330000f;
const float ACCELERATION_RATE = 0.447027f;
const float DECELERATION_RATE = 0.447027f;

void ApplySteering(CarController* controller, float input) {
    if (!controller) return;
    
    // Input processing (from disassembly patterns)
    float clampedInput = input;
    if (clampedInput > 1.0f) clampedInput = 1.0f;
    if (clampedInput < -1.0f) clampedInput = -1.0f;
    
    // Apply sensitivity
    float delta = clampedInput * STEERING_SENSITIVITY;
    float currentAngle = controller->GetSteering();
    float newAngle = currentAngle + delta;
    
    // Clamp to max angle
    if (newAngle > MAX_STEERING_ANGLE) newAngle = MAX_STEERING_ANGLE;
    if (newAngle < -MAX_STEERING_ANGLE) newAngle = -MAX_STEERING_ANGLE;
    
    controller->SetSteering(newAngle);
}

void ApplyThrottle(CarController* controller, float throttle) {
    if (!controller) return;
    
    // Clamp throttle
    if (throttle > 1.0f) throttle = 1.0f;
    if (throttle < 0.0f) throttle = 0.0f;
    
    controller->SetThrottle(throttle);
}

void ApplyBrake(CarController* controller, float brake) {
    if (!controller) return;
    
    // Clamp brake
    if (brake > 1.0f) brake = 1.0f;
    if (brake < 0.0f) brake = 0.0f;
    
    controller->SetBrake(brake);
}

void UpdateCarControl(CarController* controller, float deltaTime) {
    if (!controller) return;
    
    float throttle = controller->GetThrottle();
    float brake = controller->GetBrake();
    float steering = controller->GetSteering();
    
    // Physics update based on binary constants
    // This logic is inferred from the disassembly
    
    // Steering affects speed
    float steeringDrag = 1.0f - (fabs(steering) * 0.1f);
    
    // Apply acceleration/deceleration
    // Constants from binary: 0.330000 (friction), 0.447027 (accel/decel)
    
    // Natural deceleration when no input
    if (throttle == 0.0f && brake == 0.0f) {
        // Friction applies
    }
}

} // namespace CarControlLogic

