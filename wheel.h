// Wheel Class - Detailed Implementation
// Extracted from SPEED2.EXE
// Individual wheel representation for vehicle physics

#pragma once

#include <windows.h>
#include <cmath>
#include <string>

// Forward declarations
class Vehicle;

/**
 * Wheel - Represents a single wheel on a vehicle
 * 
 * Found in binary:
 * - CAR_WHEEL_FL (Front Left) at offset 0x385FA4
 * - CAR_WHEEL_FR (Front Right) at offset 0x385F94
 * - CAR_WHEEL_RL (Rear Left) at offset 0x385F74
 * - CAR_WHEEL_RR (Rear Right) at offset 0x385F84
 * - SFXCTL_3DLeftWheelPos, SFXCTL_3DRightWheelPos (3D positions)
 * - HUD_FEATURE_TIRE_FORCES (tire force visualization)
 * 
 * Provides:
 * - Position and rotation tracking
 * - Slip calculations (lateral and forward)
 * - Tire forces
 * - Suspension simulation
 * - Visual representation
 */
class Wheel {
public:
    enum class Position {
        FrontLeft,   // FL
        FrontRight,  // FR
        RearLeft,    // RL
        RearRight    // RR
    };
    
    Wheel(Position position);
    ~Wheel();
    
    // ============================================
    // Initialization
    // ============================================
    
    /**
     * Initialize wheel
     */
    void Initialize();
    
    /**
     * Shutdown wheel
     */
    void Shutdown();
    
    // ============================================
    // Main Update
    // ============================================
    
    /**
     * Update wheel
     * Called every frame
     */
    void Update(float deltaTime);
    
    // ============================================
    // Position and Transform
    // ============================================
    
    /**
     * Set world position
     */
    void SetPosition(float x, float y, float z);
    void GetPosition(float& x, float& y, float& z) const;
    
    /**
     * Set local position (relative to vehicle)
     */
    void SetLocalPosition(float x, float y, float z);
    void GetLocalPosition(float& x, float& y, float& z) const;
    
    /**
     * Set rotation (steering angle, camber, etc.)
     */
    void SetRotation(float steeringAngle, float camber = 0.0f, float caster = 0.0f);
    void GetRotation(float& steeringAngle, float& camber, float& caster) const;
    
    /**
     * Set steering angle
     */
    void SetSteeringAngle(float angle);
    float GetSteeringAngle() const { return m_steeringAngle; }
    
    /**
     * Set wheel rotation (for spinning wheel effect)
     */
    void SetWheelRotation(float rotation);
    float GetWheelRotation() const { return m_wheelRotation; }
    
    /**
     * Get wheel position
     */
    Position GetPosition() const { return m_position; }
    
    // ============================================
    // Slip and Forces
    // ============================================
    
    /**
     * Get lateral slip
     */
    float GetLateralSlip() const { return m_lateralSlip; }
    
    /**
     * Get forward slip
     */
    float GetForwardSlip() const { return m_forwardSlip; }
    
    /**
     * Get total slip
     */
    float GetTotalSlip() const;
    
    /**
     * Get slip angle
     */
    float GetSlipAngle() const;
    
    /**
     * Set lateral slip
     */
    void SetLateralSlip(float slip) { m_lateralSlip = slip; }
    
    /**
     * Set forward slip
     */
    void SetForwardSlip(float slip) { m_forwardSlip = slip; }
    
    /**
     * Get tire force (lateral)
     */
    float GetLateralForce() const { return m_lateralForce; }
    
    /**
     * Get tire force (forward)
     */
    float GetForwardForce() const { return m_forwardForce; }
    
    /**
     * Get normal force (weight on wheel)
     */
    float GetNormalForce() const { return m_normalForce; }
    
    /**
     * Set normal force
     */
    void SetNormalForce(float force) { m_normalForce = force; }
    
    // ============================================
    // Suspension
    // ============================================
    
    /**
     * Get suspension compression (0.0 to 1.0)
     */
    float GetSuspensionCompression() const { return m_suspensionCompression; }
    
    /**
     * Set suspension compression
     */
    void SetSuspensionCompression(float compression);
    
    /**
     * Get suspension travel
     */
    float GetSuspensionTravel() const { return m_suspensionTravel; }
    
    /**
     * Check if wheel is on ground
     */
    bool IsOnGround() const { return m_onGround; }
    
    /**
     * Set on ground state
     */
    void SetOnGround(bool onGround) { m_onGround = onGround; }
    
    // ============================================
    // Physics
    // ============================================
    
    /**
     * Get angular velocity (wheel spin)
     */
    float GetAngularVelocity() const { return m_angularVelocity; }
    
    /**
     * Set angular velocity
     */
    void SetAngularVelocity(float velocity) { m_angularVelocity = velocity; }
    
    /**
     * Get radius
     */
    float GetRadius() const { return m_radius; }
    
    /**
     * Set radius
     */
    void SetRadius(float radius) { m_radius = radius; }
    
    /**
     * Get width
     */
    float GetWidth() const { return m_width; }
    
    /**
     * Set width
     */
    void SetWidth(float width) { m_width = width; }
    
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
    
    // ============================================
    // Visual
    // ============================================
    
    /**
     * Set model name
     */
    void SetModelName(const std::string& modelName) { m_modelName = modelName; }
    const std::string& GetModelName() const { return m_modelName; }
    
    /**
     * Set visible
     */
    void SetVisible(bool visible) { m_visible = visible; }
    bool IsVisible() const { return m_visible; }

private:
    // Position
    Position m_position;
    
    // World position
    float m_x = 0.0f;
    float m_y = 0.0f;
    float m_z = 0.0f;
    
    // Local position (relative to vehicle)
    float m_localX = 0.0f;
    float m_localY = 0.0f;
    float m_localZ = 0.0f;
    
    // Rotation
    float m_steeringAngle = 0.0f;  // Steering angle in radians
    float m_camber = 0.0f;         // Camber angle in radians
    float m_caster = 0.0f;         // Caster angle in radians
    float m_wheelRotation = 0.0f;  // Wheel spin rotation
    
    // Slip
    float m_lateralSlip = 0.0f;     // Lateral (sideways) slip
    float m_forwardSlip = 0.0f;    // Forward/backward slip
    
    // Forces
    float m_lateralForce = 0.0f;   // Lateral tire force
    float m_forwardForce = 0.0f;   // Forward tire force
    float m_normalForce = 0.0f;    // Normal force (weight on wheel)
    
    // Suspension
    float m_suspensionCompression = 0.0f;  // 0.0 = fully extended, 1.0 = fully compressed
    float m_suspensionTravel = 0.0f;      // Suspension travel distance
    bool m_onGround = false;               // Is wheel touching ground
    
    // Physics
    float m_angularVelocity = 0.0f;  // Angular velocity (rad/s)
    float m_radius = 0.3f;           // Wheel radius in meters
    float m_width = 0.2f;            // Wheel width in meters
    
    // Vehicle reference
    Vehicle* m_vehicle = nullptr;
    
    // Visual
    std::string m_modelName;
    bool m_visible = true;
    
    // Timing
    float m_totalTime = 0.0f;
    
    // Constants
    static constexpr float MAX_STEERING_ANGLE = 0.785f;  // ~45 degrees (radians)
    static constexpr float MAX_CAMBER = 0.174f;          // ~10 degrees (radians)
    static constexpr float MAX_SUSPENSION_TRAVEL = 0.2f; // 20cm travel
    
    // Internal methods
    void UpdatePosition();
    void UpdateRotation(float deltaTime);
    void UpdateSlip(float deltaTime);
    void UpdateForces(float deltaTime);
    void UpdateSuspension(float deltaTime);
    void ClampSteeringAngle();
};

// ============================================
// Wheel Implementation
// ============================================

inline Wheel::Wheel(Position position) : m_position(position) {
    m_x = 0.0f;
    m_y = 0.0f;
    m_z = 0.0f;
    m_localX = 0.0f;
    m_localY = 0.0f;
    m_localZ = 0.0f;
    m_steeringAngle = 0.0f;
    m_camber = 0.0f;
    m_caster = 0.0f;
    m_wheelRotation = 0.0f;
    m_lateralSlip = 0.0f;
    m_forwardSlip = 0.0f;
    m_lateralForce = 0.0f;
    m_forwardForce = 0.0f;
    m_normalForce = 0.0f;
    m_suspensionCompression = 0.0f;
    m_suspensionTravel = 0.0f;
    m_onGround = false;
    m_angularVelocity = 0.0f;
    m_radius = 0.3f;
    m_width = 0.2f;
    m_vehicle = nullptr;
    m_visible = true;
    m_totalTime = 0.0f;
}

inline Wheel::~Wheel() {
    Shutdown();
}

inline void Wheel::Initialize() {
    m_lateralSlip = 0.0f;
    m_forwardSlip = 0.0f;
    m_lateralForce = 0.0f;
    m_forwardForce = 0.0f;
    m_normalForce = 0.0f;
    m_suspensionCompression = 0.0f;
    m_suspensionTravel = 0.0f;
    m_onGround = false;
    m_angularVelocity = 0.0f;
    m_wheelRotation = 0.0f;
    m_totalTime = 0.0f;
}

inline void Wheel::Shutdown() {
    m_vehicle = nullptr;
}

inline void Wheel::Update(float deltaTime) {
    m_totalTime += deltaTime;
    
    // Update position
    UpdatePosition();
    
    // Update rotation
    UpdateRotation(deltaTime);
    
    // Update slip
    UpdateSlip(deltaTime);
    
    // Update forces
    UpdateForces(deltaTime);
    
    // Update suspension
    UpdateSuspension(deltaTime);
}

inline void Wheel::SetPosition(float x, float y, float z) {
    m_x = x;
    m_y = y;
    m_z = z;
}

inline void Wheel::GetPosition(float& x, float& y, float& z) const {
    x = m_x;
    y = m_y;
    z = m_z;
}

inline void Wheel::SetLocalPosition(float x, float y, float z) {
    m_localX = x;
    m_localY = y;
    m_localZ = z;
}

inline void Wheel::GetLocalPosition(float& x, float& y, float& z) const {
    x = m_localX;
    y = m_localY;
    z = m_localZ;
}

inline void Wheel::SetRotation(float steeringAngle, float camber, float caster) {
    m_steeringAngle = steeringAngle;
    m_camber = camber;
    m_caster = caster;
    ClampSteeringAngle();
}

inline void Wheel::GetRotation(float& steeringAngle, float& camber, float& caster) const {
    steeringAngle = m_steeringAngle;
    camber = m_camber;
    caster = m_caster;
}

inline void Wheel::SetSteeringAngle(float angle) {
    m_steeringAngle = angle;
    ClampSteeringAngle();
}

inline void Wheel::SetWheelRotation(float rotation) {
    m_wheelRotation = rotation;
}

inline float Wheel::GetTotalSlip() const {
    return std::sqrt(m_lateralSlip * m_lateralSlip + m_forwardSlip * m_forwardSlip);
}

inline float Wheel::GetSlipAngle() const {
    if (m_forwardSlip < 0.001f) return 0.0f;
    return std::atan2(m_lateralSlip, m_forwardSlip);
}

inline void Wheel::SetSuspensionCompression(float compression) {
    m_suspensionCompression = std::max(0.0f, std::min(1.0f, compression));
    m_suspensionTravel = m_suspensionCompression * MAX_SUSPENSION_TRAVEL;
}

inline void Wheel::UpdatePosition() {
    // Update world position from local position and vehicle transform
    if (m_vehicle) {
        float vx, vy, vz;
        m_vehicle->GetPosition(vx, vy, vz);
        
        // Apply vehicle rotation to local position
        float heading = m_vehicle->GetHeading();
        float cosH = std::cos(heading);
        float sinH = std::sin(heading);
        
        // Rotate local position
        m_x = vx + m_localX * cosH - m_localZ * sinH;
        m_y = vy + m_localY;
        m_z = vz + m_localX * sinH + m_localZ * cosH;
    }
}

inline void Wheel::UpdateRotation(float deltaTime) {
    // Update wheel rotation based on angular velocity
    m_wheelRotation += m_angularVelocity * deltaTime;
    
    // Wrap rotation
    if (m_wheelRotation > 2.0f * 3.14159f) {
        m_wheelRotation -= 2.0f * 3.14159f;
    } else if (m_wheelRotation < 0.0f) {
        m_wheelRotation += 2.0f * 3.14159f;
    }
}

inline void Wheel::UpdateSlip(float deltaTime) {
    // Slip is updated by WheelSlipController
    // This method can be used for additional slip calculations
}

inline void Wheel::UpdateForces(float deltaTime) {
    // Calculate tire forces based on slip
    // Simplified tire model
    
    // Lateral force (proportional to lateral slip)
    float maxLateralForce = m_normalForce * 1.2f;  // Friction coefficient
    m_lateralForce = m_lateralSlip * maxLateralForce;
    
    // Forward force (proportional to forward slip)
    float maxForwardForce = m_normalForce * 1.2f;
    m_forwardForce = m_forwardSlip * maxForwardForce;
    
    // Clamp forces
    m_lateralForce = std::max(-maxLateralForce, std::min(maxLateralForce, m_lateralForce));
    m_forwardForce = std::max(-maxForwardForce, std::min(maxForwardForce, m_forwardForce));
}

inline void Wheel::UpdateSuspension(float deltaTime) {
    // Simple suspension simulation
    // In a full implementation, this would simulate spring/damper
    
    if (m_onGround) {
        // Compress suspension based on ground contact
        float targetCompression = 0.5f;  // Target compression when on ground
        float lerpSpeed = 5.0f * deltaTime;
        m_suspensionCompression = m_suspensionCompression * (1.0f - lerpSpeed) + targetCompression * lerpSpeed;
    } else {
        // Extend suspension when not on ground
        float lerpSpeed = 3.0f * deltaTime;
        m_suspensionCompression = m_suspensionCompression * (1.0f - lerpSpeed);
    }
    
    m_suspensionTravel = m_suspensionCompression * MAX_SUSPENSION_TRAVEL;
}

inline void Wheel::ClampSteeringAngle() {
    m_steeringAngle = std::max(-MAX_STEERING_ANGLE, std::min(MAX_STEERING_ANGLE, m_steeringAngle));
}

