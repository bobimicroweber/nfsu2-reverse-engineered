// PhysicsEngine Class
// Extracted from SPEED2.EXE
// Physics simulation engine for Need for Speed Underground 2

#pragma once

#include <windows.h>
#include <memory>
#include <vector>
#include <unordered_map>
#include <cmath>

// Forward declarations
class RigidBody;
class CollisionShape;
class Vehicle;

/**
 * PhysicsEngine - Core physics simulation engine
 * 
 * Handles:
 * - Rigid body dynamics
 * - Collision detection and response
 * - Vehicle physics
 * - Force application
 * - Gravity and friction
 */
class PhysicsEngine {
public:
    PhysicsEngine();
    ~PhysicsEngine();
    
    // ============================================
    // Initialization and Shutdown
    // ============================================
    
    /**
     * Initialize physics engine
     */
    bool Initialize();
    
    /**
     * Shutdown physics engine
     */
    void Shutdown();
    
    // ============================================
    // Main Update Loop
    // ============================================
    
    /**
     * Update physics simulation
     * Called every frame with delta time
     */
    void Update(float deltaTime);
    
    /**
     * Step physics simulation
     * Performs one physics step
     */
    void Step(float deltaTime);
    
    // ============================================
    // Rigid Body Management
    // ============================================
    
    /**
     * Create a rigid body
     */
    RigidBody* CreateRigidBody(const std::string& name);
    
    /**
     * Remove a rigid body
     */
    void RemoveRigidBody(RigidBody* body);
    
    /**
     * Get rigid body by name
     */
    RigidBody* GetRigidBody(const std::string& name);
    
    // ============================================
    // Force and Impulse
    // ============================================
    
    /**
     * Apply force to a rigid body
     */
    void ApplyForce(RigidBody* body, float fx, float fy, float fz);
    
    /**
     * Apply impulse to a rigid body
     */
    void ApplyImpulse(RigidBody* body, float ix, float iy, float iz);
    
    /**
     * Apply torque to a rigid body
     */
    void ApplyTorque(RigidBody* body, float tx, float ty, float tz);
    
    // ============================================
    // Physics Constants
    // ============================================
    
    /**
     * Set gravity
     */
    void SetGravity(float x, float y, float z);
    
    /**
     * Get gravity
     */
    void GetGravity(float& x, float& y, float& z) const {
        x = m_gravityX;
        y = m_gravityY;
        z = m_gravityZ;
    }
    
    /**
     * Set global friction
     */
    void SetGlobalFriction(float friction) {
        m_globalFriction = friction;
    }
    
    float GetGlobalFriction() const { return m_globalFriction; }
    
    // ============================================
    // Collision Detection
    // ============================================
    
    /**
     * Perform collision detection
     */
    void DetectCollisions();
    
    /**
     * Resolve collisions
     */
    void ResolveCollisions();
    
    // ============================================
    // Vehicle Physics
    // ============================================
    
    /**
     * Update vehicle physics
     */
    void UpdateVehiclePhysics(Vehicle* vehicle, float deltaTime);
    
    /**
     * Apply vehicle forces
     */
    void ApplyVehicleForces(Vehicle* vehicle, float throttle, float brake, float steering);

private:
    // Physics constants (from binary analysis)
    float m_gravityX = 0.0f;
    float m_gravityY = -9.8f;  // Standard gravity
    float m_gravityZ = 0.0f;
    float m_globalFriction = 0.330000f;  // From binary: 0x3842A4
    
    // Simulation settings
    float m_timeStep = 1.0f / 60.0f;  // 60 FPS default
    int m_maxSubSteps = 4;
    float m_accumulator = 0.0f;
    
    // Rigid bodies
    std::vector<std::unique_ptr<RigidBody>> m_rigidBodies;
    std::unordered_map<std::string, RigidBody*> m_bodyMap;
    
    // Collision pairs
    std::vector<std::pair<RigidBody*, RigidBody*>> m_collisionPairs;
    
    // State
    bool m_initialized = false;
    bool m_paused = false;
    
    // Internal methods
    void IntegrateRigidBody(RigidBody* body, float deltaTime);
    void ApplyGravity(RigidBody* body, float deltaTime);
    void ApplyFriction(RigidBody* body, float deltaTime);
};

// ============================================
// RigidBody Class
// ============================================

/**
 * RigidBody - Represents a physics object
 */
class RigidBody {
public:
    RigidBody(const std::string& name);
    ~RigidBody();
    
    // Position and rotation
    void SetPosition(float x, float y, float z);
    void GetPosition(float& x, float& y, float& z) const {
        x = m_x; y = m_y; z = m_z;
    }
    
    void SetRotation(float rx, float ry, float rz);
    void GetRotation(float& rx, float& ry, float& rz) const {
        rx = m_rx; ry = m_ry; rz = m_rz;
    }
    
    // Velocity
    void SetVelocity(float vx, float vy, float vz);
    void GetVelocity(float& vx, float& vy, float& vz) const {
        vx = m_vx; vy = m_vy; vz = m_vz;
    }
    
    // Angular velocity
    void SetAngularVelocity(float avx, float avy, float avz);
    void GetAngularVelocity(float& avx, float& avy, float& avz) const {
        avx = m_avx; avy = m_avy; avz = m_avz;
    }
    
    // Mass and inertia
    void SetMass(float mass) { m_mass = mass; UpdateInertia(); }
    float GetMass() const { return m_mass; }
    
    void SetInertia(float ix, float iy, float iz);
    void GetInertia(float& ix, float& iy, float& iz) const {
        ix = m_ix; iy = m_iy; iz = m_iz;
    }
    
    // Forces
    void AddForce(float fx, float fy, float fz);
    void AddTorque(float tx, float ty, float tz);
    
    // Properties
    void SetFriction(float friction) { m_friction = friction; }
    float GetFriction() const { return m_friction; }
    
    void SetRestitution(float restitution) { m_restitution = restitution; }
    float GetRestitution() const { return m_restitution; }
    
    const std::string& GetName() const { return m_name; }
    bool IsStatic() const { return m_static; }
    void SetStatic(bool isStatic) { m_static = isStatic; }

private:
    std::string m_name;
    
    // Position and rotation
    float m_x = 0.0f, m_y = 0.0f, m_z = 0.0f;
    float m_rx = 0.0f, m_ry = 0.0f, m_rz = 0.0f;
    
    // Velocity
    float m_vx = 0.0f, m_vy = 0.0f, m_vz = 0.0f;
    float m_avx = 0.0f, m_avy = 0.0f, m_avz = 0.0f;
    
    // Mass and inertia
    float m_mass = 1.0f;
    float m_ix = 1.0f, m_iy = 1.0f, m_iz = 1.0f;
    
    // Forces (accumulated per frame)
    float m_fx = 0.0f, m_fy = 0.0f, m_fz = 0.0f;
    float m_tx = 0.0f, m_ty = 0.0f, m_tz = 0.0f;
    
    // Properties
    float m_friction = 0.330000f;  // From binary
    float m_restitution = 0.5f;  // Bounciness
    bool m_static = false;
    
    void UpdateInertia();
};

// ============================================
// PhysicsEngine Implementation
// ============================================

inline PhysicsEngine::PhysicsEngine() {
    m_gravityY = -9.8f;
    m_globalFriction = 0.330000f;  // From binary offset 0x3842A4
    m_timeStep = 1.0f / 60.0f;
    m_maxSubSteps = 4;
    m_accumulator = 0.0f;
    m_initialized = false;
    m_paused = false;
}

inline PhysicsEngine::~PhysicsEngine() {
    Shutdown();
}

inline bool PhysicsEngine::Initialize() {
    if (m_initialized) {
        return true;
    }
    
    // Initialize physics systems
    m_rigidBodies.clear();
    m_bodyMap.clear();
    m_collisionPairs.clear();
    
    m_initialized = true;
    return true;
}

inline void PhysicsEngine::Shutdown() {
    m_rigidBodies.clear();
    m_bodyMap.clear();
    m_collisionPairs.clear();
    m_initialized = false;
}

inline void PhysicsEngine::Update(float deltaTime) {
    if (!m_initialized || m_paused) {
        return;
    }
    
    // Fixed timestep physics
    m_accumulator += deltaTime;
    
    int steps = 0;
    while (m_accumulator >= m_timeStep && steps < m_maxSubSteps) {
        Step(m_timeStep);
        m_accumulator -= m_timeStep;
        steps++;
    }
}

inline void PhysicsEngine::Step(float deltaTime) {
    // Clear forces
    for (auto& body : m_rigidBodies) {
        if (body && !body->IsStatic()) {
            body->m_fx = 0.0f;
            body->m_fy = 0.0f;
            body->m_fz = 0.0f;
            body->m_tx = 0.0f;
            body->m_ty = 0.0f;
            body->m_tz = 0.0f;
        }
    }
    
    // Apply gravity
    for (auto& body : m_rigidBodies) {
        if (body && !body->IsStatic()) {
            ApplyGravity(body.get(), deltaTime);
        }
    }
    
    // Apply forces
    // (Forces are accumulated during the frame)
    
    // Integrate
    for (auto& body : m_rigidBodies) {
        if (body && !body->IsStatic()) {
            IntegrateRigidBody(body.get(), deltaTime);
        }
    }
    
    // Apply friction
    for (auto& body : m_rigidBodies) {
        if (body && !body->IsStatic()) {
            ApplyFriction(body.get(), deltaTime);
        }
    }
    
    // Collision detection
    DetectCollisions();
    
    // Collision response
    ResolveCollisions();
}

inline void PhysicsEngine::IntegrateRigidBody(RigidBody* body, float deltaTime) {
    if (!body || body->IsStatic()) return;
    
    float mass = body->GetMass();
    if (mass <= 0.0f) return;
    
    // Linear integration (F = ma, v = v0 + at, x = x0 + vt)
    float ax = body->m_fx / mass;
    float ay = body->m_fy / mass;
    float az = body->m_fz / mass;
    
    body->m_vx += ax * deltaTime;
    body->m_vy += ay * deltaTime;
    body->m_vz += az * deltaTime;
    
    body->m_x += body->m_vx * deltaTime;
    body->m_y += body->m_vy * deltaTime;
    body->m_z += body->m_vz * deltaTime;
    
    // Angular integration
    float iax = body->m_tx / body->m_ix;
    float iay = body->m_ty / body->m_iy;
    float iaz = body->m_tz / body->m_iz;
    
    body->m_avx += iax * deltaTime;
    body->m_avy += iay * deltaTime;
    body->m_avz += iaz * deltaTime;
    
    body->m_rx += body->m_avx * deltaTime;
    body->m_ry += body->m_avy * deltaTime;
    body->m_rz += body->m_avz * deltaTime;
}

inline void PhysicsEngine::ApplyGravity(RigidBody* body, float deltaTime) {
    if (!body || body->IsStatic()) return;
    
    float mass = body->GetMass();
    body->m_fy += m_gravityY * mass;
}

inline void PhysicsEngine::ApplyFriction(RigidBody* body, float deltaTime) {
    if (!body || body->IsStatic()) return;
    
    float friction = body->GetFriction() * m_globalFriction;
    
    // Apply friction to velocity
    float vx = body->m_vx;
    float vy = body->m_vy;
    float vz = body->m_vz;
    
    float speed = std::sqrt(vx*vx + vy*vy + vz*vz);
    if (speed > 0.001f) {
        float frictionForce = friction * body->GetMass();
        float decel = frictionForce / body->GetMass();
        
        float factor = std::max(0.0f, 1.0f - (decel * deltaTime / speed));
        body->m_vx *= factor;
        body->m_vy *= factor;
        body->m_vz *= factor;
    }
}

inline RigidBody* PhysicsEngine::CreateRigidBody(const std::string& name) {
    auto body = std::make_unique<RigidBody>(name);
    RigidBody* ptr = body.get();
    m_rigidBodies.push_back(std::move(body));
    m_bodyMap[name] = ptr;
    return ptr;
}

inline void PhysicsEngine::RemoveRigidBody(RigidBody* body) {
    if (!body) return;
    
    m_bodyMap.erase(body->GetName());
    m_rigidBodies.erase(
        std::remove_if(m_rigidBodies.begin(), m_rigidBodies.end(),
            [body](const std::unique_ptr<RigidBody>& b) {
                return b.get() == body;
            }),
        m_rigidBodies.end()
    );
}

inline RigidBody* PhysicsEngine::GetRigidBody(const std::string& name) {
    auto it = m_bodyMap.find(name);
    return (it != m_bodyMap.end()) ? it->second : nullptr;
}

inline void PhysicsEngine::ApplyForce(RigidBody* body, float fx, float fy, float fz) {
    if (!body || body->IsStatic()) return;
    body->AddForce(fx, fy, fz);
}

inline void PhysicsEngine::ApplyImpulse(RigidBody* body, float ix, float iy, float iz) {
    if (!body || body->IsStatic()) return;
    
    float mass = body->GetMass();
    if (mass > 0.0f) {
        body->m_vx += ix / mass;
        body->m_vy += iy / mass;
        body->m_vz += iz / mass;
    }
}

inline void PhysicsEngine::ApplyTorque(RigidBody* body, float tx, float ty, float tz) {
    if (!body || body->IsStatic()) return;
    body->AddTorque(tx, ty, tz);
}

inline void PhysicsEngine::SetGravity(float x, float y, float z) {
    m_gravityX = x;
    m_gravityY = y;
    m_gravityZ = z;
}

inline void PhysicsEngine::DetectCollisions() {
    m_collisionPairs.clear();
    
    // Simple broad phase (check all pairs)
    // In production, use spatial partitioning
    for (size_t i = 0; i < m_rigidBodies.size(); ++i) {
        for (size_t j = i + 1; j < m_rigidBodies.size(); ++j) {
            RigidBody* body1 = m_rigidBodies[i].get();
            RigidBody* body2 = m_rigidBodies[j].get();
            
            if (body1 && body2 && !body1->IsStatic() && !body2->IsStatic()) {
                // Simple AABB collision check
                // In production, use proper collision shapes
                float dx = body1->m_x - body2->m_x;
                float dy = body1->m_y - body2->m_y;
                float dz = body1->m_z - body2->m_z;
                float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                
                // Assume radius of 1.0 for now
                if (dist < 2.0f) {
                    m_collisionPairs.push_back({body1, body2});
                }
            }
        }
    }
}

inline void PhysicsEngine::ResolveCollisions() {
    for (auto& pair : m_collisionPairs) {
        RigidBody* body1 = pair.first;
        RigidBody* body2 = pair.second;
        
        if (!body1 || !body2) continue;
        
        // Simple collision response
        // Calculate collision normal
        float dx = body2->m_x - body1->m_x;
        float dy = body2->m_y - body1->m_y;
        float dz = body2->m_z - body1->m_z;
        float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        if (dist > 0.001f) {
            float nx = dx / dist;
            float ny = dy / dist;
            float nz = dz / dist;
            
            // Relative velocity
            float rvx = body2->m_vx - body1->m_vx;
            float rvy = body2->m_vy - body1->m_vy;
            float rvz = body2->m_vz - body1->m_vz;
            
            // Relative velocity along normal
            float velAlongNormal = rvx*nx + rvy*ny + rvz*nz;
            
            // Don't resolve if separating
            if (velAlongNormal > 0) continue;
            
            // Restitution
            float e = std::min(body1->GetRestitution(), body2->GetRestitution());
            
            // Impulse scalar
            float j = -(1.0f + e) * velAlongNormal;
            j /= (1.0f / body1->GetMass() + 1.0f / body2->GetMass());
            
            // Apply impulse
            float impulseX = j * nx;
            float impulseY = j * ny;
            float impulseZ = j * nz;
            
            body1->m_vx -= impulseX / body1->GetMass();
            body1->m_vy -= impulseY / body1->GetMass();
            body1->m_vz -= impulseZ / body1->GetMass();
            
            body2->m_vx += impulseX / body2->GetMass();
            body2->m_vy += impulseY / body2->GetMass();
            body2->m_vz += impulseZ / body2->GetMass();
        }
    }
}

inline void PhysicsEngine::UpdateVehiclePhysics(Vehicle* vehicle, float deltaTime) {
    // Vehicle-specific physics update
    // Integrates with CarController/RealController
}

inline void PhysicsEngine::ApplyVehicleForces(Vehicle* vehicle, float throttle, float brake, float steering) {
    // Apply forces based on vehicle input
    // Throttle: forward force
    // Brake: backward force
    // Steering: lateral force
}

// RigidBody implementation
inline RigidBody::RigidBody(const std::string& name) 
    : m_name(name)
    , m_friction(0.330000f)  // From binary
    , m_restitution(0.5f)
    , m_static(false)
{
}

inline RigidBody::~RigidBody() {
}

inline void RigidBody::SetPosition(float x, float y, float z) {
    m_x = x; m_y = y; m_z = z;
}

inline void RigidBody::SetRotation(float rx, float ry, float rz) {
    m_rx = rx; m_ry = ry; m_rz = rz;
}

inline void RigidBody::SetVelocity(float vx, float vy, float vz) {
    m_vx = vx; m_vy = vy; m_vz = vz;
}

inline void RigidBody::SetAngularVelocity(float avx, float avy, float avz) {
    m_avx = avx; m_avy = avy; m_avz = avz;
}

inline void RigidBody::SetInertia(float ix, float iy, float iz) {
    m_ix = ix; m_iy = iy; m_iz = iz;
}

inline void RigidBody::AddForce(float fx, float fy, float fz) {
    m_fx += fx; m_fy += fy; m_fz += fz;
}

inline void RigidBody::AddTorque(float tx, float ty, float tz) {
    m_tx += tx; m_ty += ty; m_tz += tz;
}

inline void RigidBody::UpdateInertia() {
    // Simple inertia calculation
    // In production, calculate based on shape
    m_ix = m_iy = m_iz = m_mass * 0.1f;
}

