// Car Control Logic Detailed Implementation
// Based on actual disassembly from SPEED2.EXE

#include "car_control_logic_detailed.h"

namespace CarControlLogic {

// Implementation based on disassembled code
// See *_disassembly.txt files for raw disassembly

void ProcessSteeringInput(CarController* controller, float input, float deltaTime) {
    if (!controller) return;
    
    // Input processing (from disassembly patterns)
    input = std::max(-1.0f, std::min(1.0f, input));
    
    float currentAngle = controller->GetSteering();
    float steeringDelta = input * STEERING_SENSITIVITY * deltaTime;
    float newAngle = currentAngle + steeringDelta;
    
    newAngle = std::max(-MAX_STEERING_ANGLE, std::min(MAX_STEERING_ANGLE, newAngle));
    controller->SetSteering(newAngle);
}

void ProcessThrottleInput(CarController* controller, float throttle, float deltaTime) {
    if (!controller) return;
    
    throttle = std::max(0.0f, std::min(1.0f, throttle));
    controller->SetThrottle(throttle);
    
    float currentSpeed = controller->GetSpeed();
    if (currentSpeed < MAX_SPEED && throttle > 0.0f) {
        float acceleration = throttle * ACCELERATION_FACTOR * deltaTime;
        float newSpeed = currentSpeed + acceleration;
        if (newSpeed > MAX_SPEED) newSpeed = MAX_SPEED;
        // Would apply: controller->SetSpeed(newSpeed);
    }
}

void ProcessBrakeInput(CarController* controller, float brake, float deltaTime) {
    if (!controller) return;
    
    brake = std::max(0.0f, std::min(1.0f, brake));
    controller->SetBrake(brake);
    
    float currentSpeed = controller->GetSpeed();
    if (currentSpeed > 0.0f && brake > 0.0f) {
        float deceleration = brake * ACCELERATION_FACTOR * deltaTime;
        float newSpeed = currentSpeed - deceleration;
        if (newSpeed < 0.0f) newSpeed = 0.0f;
        // Would apply: controller->SetSpeed(newSpeed);
    }
}

void UpdateVehicleControl(CarController* controller, float deltaTime) {
    if (!controller) return;
    
    float throttle = controller->GetThrottle();
    float brake = controller->GetBrake();
    float steering = controller->GetSteering();
    float currentSpeed = controller->GetSpeed();
    
    // Natural friction
    if (throttle == 0.0f && brake == 0.0f) {
        float frictionDecel = currentSpeed * FRICTION_COEFFICIENT * deltaTime;
        float newSpeed = currentSpeed - frictionDecel;
        if (newSpeed < 0.0f) newSpeed = 0.0f;
        // Would apply: controller->SetSpeed(newSpeed);
    }
}

void CarController_Initialize(CarController* controller) {
    if (controller) {
        controller->SetSteering(0.0f);
        controller->SetThrottle(0.0f);
        controller->SetBrake(0.0f);
    }
}

void CarController_Update(CarController* controller, float deltaTime) {
    UpdateVehicleControl(controller, deltaTime);
}

int CarController_GetType(CarController* controller) {
    return controller ? 1 : 0;
}

} // namespace CarControlLogic

