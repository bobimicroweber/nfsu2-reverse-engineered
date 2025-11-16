// Vehicle Class - Detailed Implementation
// Extracted from SPEED2.EXE
// Comprehensive vehicle representation with physics, rendering, and control

#pragma once

#include <windows.h>
#include <cmath>
#include <string>
#include <memory>
#include <vector>

// Forward declarations
class CarController;
class RealController;
class DriftDriver;
class WheelSlipController;
class Renderer;
class PhysicsEngine;
class RigidBody;

/**
 * Vehicle - Represents a vehicle in the game
 * 
 * Provides:
 * - Position, rotation, and velocity management
 * - Physics integration
 * - Controller management (CarController, RealController, DriftDriver)
 * - Rendering support
 * - Model and visual representation
 * - Speed and movement tracking
 */
class Vehicle {
public:
    Vehicle(const std::string& name);
    ~Vehicle();
    
    // ============================================
    // Initialization
    // ============================================
    
    /**
     * Initialize vehicle
     */
    bool Initialize();
    
    /**
     * Shutdown vehicle
     */
    void Shutdown();
    
    // ============================================
    // Main Update
    // ============================================
    
    /**
     * Update vehicle
     * Called every frame
     */
    void Update(float deltaTime);
    
    /**
     * Render vehicle
     */
    void Render(Renderer* renderer);
    
    // ============================================
    // Position and Transform
    // ============================================
    
    /**
     * Set position
     */
    void SetPosition(float x, float y, float z);
    void GetPosition(float& x, float& y, float& z) const;
    
    /**
     * Set rotation (Euler angles in radians)
     */
    void SetRotation(float rx, float ry, float rz);
    void GetRotation(float& rx, float& ry, float& rz) const;
    
    /**
     * Set heading (yaw angle in radians)
     */
    void SetHeading(float heading);
    float GetHeading() const { return m_heading; }
    
    /**
     * Get forward direction vector
     */
    void GetForwardVector(float& x, float& y, float& z) const;
    
    /**
     * Get right direction vector
     */
    void GetRightVector(float& x, float& y, float& z) const;
    
    /**
     * Get up direction vector
     */
    void GetUpVector(float& x, float& y, float& z) const;
    
    // ============================================
    // Velocity and Movement
    // ============================================
    
    /**
     * Set velocity
     */
    void SetVelocity(float vx, float vy, float vz);
    void GetVelocity(float& vx, float& vy, float& vz) const;
    
    /**
     * Get current speed (magnitude of velocity)
     */
    float GetSpeed() const;
    
    /**
     * Get speed in km/h
     */
    float GetSpeedKmh() const;
    
    /**
     * Get angular velocity
     */
    void SetAngularVelocity(float avx, float avy, float avz);
    void GetAngularVelocity(float& avx, float& avy, float& avz) const;
    
    /**
     * Get yaw rate (angular velocity around Y axis)
     */
    float GetYawRate() const { return m_angularVelocityY; }
    
    // ============================================
    // Controllers
    // ============================================
    
    /**
     * Get car controller
     */
    CarController* GetController() { return m_controller.get(); }
    const CarController* GetController() const { return m_controller.get(); }
    
    /**
     * Get real controller
     */
    RealController* GetRealController() { return m_realController.get(); }
    const RealController* GetRealController() const { return m_realController.get(); }
    
    /**
     * Get drift driver
     */
    DriftDriver* GetDriftDriver() { return m_driftDriver.get(); }
    const DriftDriver* GetDriftDriver() const { return m_driftDriver.get(); }
    
    /**
     * Get wheel slip controller
     */
    WheelSlipController* GetWheelSlipController() { return m_wheelSlipController.get(); }
    const WheelSlipController* GetWheelSlipController() const { return m_wheelSlipController.get(); }
    
    /**
     * Set controller type
     */
    void SetControllerType(bool useRealistic);
    bool IsUsingRealisticController() const { return m_useRealisticController; }
    
    // ============================================
    // Physics
    // ============================================
    
    /**
     * Get rigid body for physics
     */
    RigidBody* GetRigidBody() { return m_rigidBody.get(); }
    const RigidBody* GetRigidBody() const { return m_rigidBody.get(); }
    
    /**
     * Set mass
     */
    void SetMass(float mass) { m_mass = mass; }
    float GetMass() const { return m_mass; }
    
    /**
     * Apply force
     */
    void ApplyForce(float fx, float fy, float fz);
    
    /**
     * Apply torque
     */
    void ApplyTorque(float tx, float ty, float tz);
    
    /**
     * Set friction coefficient
     */
    void SetFriction(float friction) { m_friction = friction; }
    float GetFriction() const { return m_friction; }
    
    // ============================================
    // Model and Visual
    // ============================================
    
    /**
     * Set model name/path
     */
    void SetModelName(const std::string& modelName) { m_modelName = modelName; }
    const std::string& GetModelName() const { return m_modelName; }
    
    /**
     * Set scale
     */
    void SetScale(float scale) { m_scale = scale; }
    float GetScale() const { return m_scale; }
    
    /**
     * Set visible
     */
    void SetVisible(bool visible) { m_visible = visible; }
    bool IsVisible() const { return m_visible; }
    
    // ============================================
    // Properties
    // ============================================
    
    /**
     * Get name
     */
    const std::string& GetName() const { return m_name; }
    
    /**
     * Set active
     */
    void SetActive(bool active) { m_active = active; }
    bool IsActive() const { return m_active; }
    
    /**
     * Check if vehicle is out of control
     */
    bool IsOutOfControl() const { return m_outOfControl; }
    
    /**
     * Set out of control state
     */
    void SetOutOfControl(bool outOfControl) { m_outOfControl = outOfControl; }

private:
    // Name
    std::string m_name;
    
    // Position and rotation
    float m_x = 0.0f;
    float m_y = 0.0f;
    float m_z = 0.0f;
    float m_rotationX = 0.0f;
    float m_rotationY = 0.0f;
    float m_rotationZ = 0.0f;
    float m_heading = 0.0f;  // Yaw angle (rotation around Y axis)
    
    // Velocity
    float m_velocityX = 0.0f;
    float m_velocityY = 0.0f;
    float m_velocityZ = 0.0f;
    float m_angularVelocityX = 0.0f;
    float m_angularVelocityY = 0.0f;
    float m_angularVelocityZ = 0.0f;
    
    // Physics
    float m_mass = 1500.0f;  // Default car mass in kg
    float m_friction = 0.33f;  // Friction coefficient
    std::unique_ptr<RigidBody> m_rigidBody;
    
    // Controllers
    std::unique_ptr<CarController> m_controller;
    std::unique_ptr<RealController> m_realController;
    std::unique_ptr<DriftDriver> m_driftDriver;
    std::unique_ptr<WheelSlipController> m_wheelSlipController;
    bool m_useRealisticController = false;
    
    // Model and visual
    std::string m_modelName;
    float m_scale = 1.0f;
    bool m_visible = true;
    
    // State
    bool m_active = true;
    bool m_initialized = false;
    bool m_outOfControl = false;
    
    // Timing
    float m_totalTime = 0.0f;
    
    // Internal methods
    void UpdatePhysics(float deltaTime);
    void UpdateControllers(float deltaTime);
    void UpdateTransform();
    void UpdateVelocity(float deltaTime);
    void ApplyFriction(float deltaTime);
    void ClampVelocity();
};

// ============================================
// Vehicle Implementation
// ============================================

inline Vehicle::Vehicle(const std::string& name) : m_name(name) {
    m_x = 0.0f;
    m_y = 0.0f;
    m_z = 0.0f;
    m_rotationX = 0.0f;
    m_rotationY = 0.0f;
    m_rotationZ = 0.0f;
    m_heading = 0.0f;
    m_velocityX = 0.0f;
    m_velocityY = 0.0f;
    m_velocityZ = 0.0f;
    m_angularVelocityX = 0.0f;
    m_angularVelocityY = 0.0f;
    m_angularVelocityZ = 0.0f;
    m_mass = 1500.0f;
    m_friction = 0.33f;
    m_scale = 1.0f;
    m_visible = true;
    m_active = true;
    m_initialized = false;
    m_outOfControl = false;
    m_totalTime = 0.0f;
    m_useRealisticController = false;
}

inline Vehicle::~Vehicle() {
    Shutdown();
}

inline bool Vehicle::Initialize() {
    if (m_initialized) return true;
    
    // Create controllers
    m_controller = std::make_unique<CarController>();
    m_realController = std::make_unique<RealController>();
    m_driftDriver = std::make_unique<DriftDriver>();
    m_wheelSlipController = std::make_unique<WheelSlipController>();
    
    // Initialize controllers
    if (m_controller) {
        // Initialize basic controller
    }
    if (m_realController) {
        // Initialize realistic controller
    }
    if (m_driftDriver) {
        m_driftDriver->SetVehicle(this);
        m_driftDriver->SetWheelSlipController(m_wheelSlipController.get());
        m_driftDriver->Initialize();
    }
    if (m_wheelSlipController) {
        m_wheelSlipController->SetVehicle(this);
        m_wheelSlipController->Initialize();
    }
    
    // Create rigid body for physics
    // m_rigidBody = std::make_unique<RigidBody>(m_name);
    // if (m_rigidBody) {
    //     m_rigidBody->SetMass(m_mass);
    //     m_rigidBody->SetFriction(m_friction);
    // }
    
    m_initialized = true;
    return true;
}

inline void Vehicle::Shutdown() {
    m_controller.reset();
    m_realController.reset();
    m_driftDriver.reset();
    m_wheelSlipController.reset();
    m_rigidBody.reset();
    m_initialized = false;
}

inline void Vehicle::Update(float deltaTime) {
    if (!m_active || !m_initialized) return;
    
    m_totalTime += deltaTime;
    
    // Update controllers
    UpdateControllers(deltaTime);
    
    // Update physics
    UpdatePhysics(deltaTime);
    
    // Update transform
    UpdateTransform();
    
    // Update velocity
    UpdateVelocity(deltaTime);
    
    // Apply friction
    ApplyFriction(deltaTime);
    
    // Clamp velocity
    ClampVelocity();
}

inline void Vehicle::Render(Renderer* renderer) {
    if (!m_visible || !renderer) return;
    
    // Render vehicle model
    // renderer->DrawModel(m_modelName, m_x, m_y, m_z, m_rotationX, m_rotationY, m_rotationZ, m_scale);
}

inline void Vehicle::SetPosition(float x, float y, float z) {
    m_x = x;
    m_y = y;
    m_z = z;
    
    // Update rigid body if exists
    if (m_rigidBody) {
        m_rigidBody->SetPosition(x, y, z);
    }
}

inline void Vehicle::GetPosition(float& x, float& y, float& z) const {
    x = m_x;
    y = m_y;
    z = m_z;
}

inline void Vehicle::SetRotation(float rx, float ry, float rz) {
    m_rotationX = rx;
    m_rotationY = ry;
    m_rotationZ = rz;
    m_heading = ry;  // Heading is rotation around Y axis
    
    // Update rigid body if exists
    if (m_rigidBody) {
        m_rigidBody->SetRotation(rx, ry, rz);
    }
}

inline void Vehicle::GetRotation(float& rx, float& ry, float& rz) const {
    rx = m_rotationX;
    ry = m_rotationY;
    rz = m_rotationZ;
}

inline void Vehicle::SetHeading(float heading) {
    m_heading = heading;
    m_rotationY = heading;
    
    if (m_rigidBody) {
        m_rigidBody->SetRotation(m_rotationX, m_rotationY, m_rotationZ);
    }
}

inline void Vehicle::GetForwardVector(float& x, float& y, float& z) const {
    // Forward vector based on heading
    x = std::sin(m_heading);
    y = 0.0f;
    z = std::cos(m_heading);
}

inline void Vehicle::GetRightVector(float& x, float& y, float& z) const {
    // Right vector (perpendicular to forward)
    x = std::cos(m_heading);
    y = 0.0f;
    z = -std::sin(m_heading);
}

inline void Vehicle::GetUpVector(float& x, float& y, float& z) const {
    // Up vector (always up in world space)
    x = 0.0f;
    y = 1.0f;
    z = 0.0f;
}

inline void Vehicle::SetVelocity(float vx, float vy, float vz) {
    m_velocityX = vx;
    m_velocityY = vy;
    m_velocityZ = vz;
    
    if (m_rigidBody) {
        m_rigidBody->SetVelocity(vx, vy, vz);
    }
}

inline void Vehicle::GetVelocity(float& vx, float& vy, float& vz) const {
    vx = m_velocityX;
    vy = m_velocityY;
    vz = m_velocityZ;
}

inline float Vehicle::GetSpeed() const {
    // Speed is magnitude of velocity
    return std::sqrt(m_velocityX * m_velocityX + m_velocityY * m_velocityY + m_velocityZ * m_velocityZ);
}

inline float Vehicle::GetSpeedKmh() const {
    // Convert to km/h (assuming game units are in m/s, multiply by 3.6)
    return GetSpeed() * 3.6f;
}

inline void Vehicle::SetAngularVelocity(float avx, float avy, float avz) {
    m_angularVelocityX = avx;
    m_angularVelocityY = avy;
    m_angularVelocityZ = avz;
    
    if (m_rigidBody) {
        m_rigidBody->SetAngularVelocity(avx, avy, avz);
    }
}

inline void Vehicle::GetAngularVelocity(float& avx, float& avy, float& avz) const {
    avx = m_angularVelocityX;
    avy = m_angularVelocityY;
    avz = m_angularVelocityZ;
}

inline void Vehicle::SetControllerType(bool useRealistic) {
    m_useRealisticController = useRealistic;
}

inline void Vehicle::ApplyForce(float fx, float fy, float fz) {
    // Apply force to velocity
    float deltaTime = 0.016f;  // Assume 60 FPS
    m_velocityX += (fx / m_mass) * deltaTime;
    m_velocityY += (fy / m_mass) * deltaTime;
    m_velocityZ += (fz / m_mass) * deltaTime;
    
    if (m_rigidBody) {
        m_rigidBody->AddForce(fx, fy, fz);
    }
}

inline void Vehicle::ApplyTorque(float tx, float ty, float tz) {
    // Apply torque to angular velocity
    float deltaTime = 0.016f;  // Assume 60 FPS
    m_angularVelocityX += (tx / m_mass) * deltaTime;
    m_angularVelocityY += (ty / m_mass) * deltaTime;
    m_angularVelocityZ += (tz / m_mass) * deltaTime;
    
    if (m_rigidBody) {
        m_rigidBody->AddTorque(tx, ty, tz);
    }
}

inline void Vehicle::UpdatePhysics(float deltaTime) {
    // Update physics from rigid body if exists
    if (m_rigidBody) {
        // Get updated position/velocity from physics engine
        // m_rigidBody->Update(deltaTime);
        // m_rigidBody->GetPosition(m_x, m_y, m_z);
        // m_rigidBody->GetVelocity(m_velocityX, m_velocityY, m_velocityZ);
    }
}

inline void Vehicle::UpdateControllers(float deltaTime) {
    // Update active controller
    if (m_useRealisticController && m_realController) {
        // Use realistic controller
        // m_realController->Update(deltaTime);
    } else if (m_controller) {
        // Use basic controller
        // m_controller->Update(deltaTime);
    }
    
    // Update drift driver
    if (m_driftDriver) {
        m_driftDriver->Update(deltaTime);
    }
    
    // Update wheel slip controller
    if (m_wheelSlipController) {
        float steering = 0.0f;
        float throttle = 0.0f;
        float brake = 0.0f;
        
        // Get inputs from controller
        if (m_controller) {
            // steering = m_controller->GetSteering();
            // throttle = m_controller->GetThrottle();
            // brake = m_controller->GetBrake();
        }
        
        m_wheelSlipController->Update(deltaTime, steering, throttle, brake,
                                     GetSpeed(), GetYawRate());
    }
}

inline void Vehicle::UpdateTransform() {
    // Update rotation from angular velocity
    m_rotationX += m_angularVelocityX * 0.016f;  // Assume 60 FPS
    m_rotationY += m_angularVelocityY * 0.016f;
    m_rotationZ += m_angularVelocityZ * 0.016f;
    m_heading = m_rotationY;
}

inline void Vehicle::UpdateVelocity(float deltaTime) {
    // Update position from velocity
    m_x += m_velocityX * deltaTime;
    m_y += m_velocityY * deltaTime;
    m_z += m_velocityZ * deltaTime;
}

inline void Vehicle::ApplyFriction(float deltaTime) {
    // Apply friction to velocity
    float frictionForce = m_friction * 9.8f * m_mass;  // Friction = μ * g * m
    float frictionAccel = frictionForce / m_mass;
    
    // Apply friction in opposite direction of velocity
    float speed = GetSpeed();
    if (speed > 0.001f) {
        float frictionX = -(m_velocityX / speed) * frictionAccel * deltaTime;
        float frictionY = -(m_velocityY / speed) * frictionAccel * deltaTime;
        float frictionZ = -(m_velocityZ / speed) * frictionAccel * deltaTime;
        
        m_velocityX += frictionX;
        m_velocityY += frictionY;
        m_velocityZ += frictionZ;
    }
}

inline void Vehicle::ClampVelocity() {
    // Clamp velocity to reasonable limits
    float maxSpeed = 250.0f;  // Max speed in game units
    float speed = GetSpeed();
    if (speed > maxSpeed) {
        float scale = maxSpeed / speed;
        m_velocityX *= scale;
        m_velocityY *= scale;
        m_velocityZ *= scale;
    }
}

