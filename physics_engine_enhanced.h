// PhysicsEngine Enhanced - With Collision Bodies
// Extracted from SPEED2.EXE
// Includes collision body types found in binary

#pragma once

#include "physics_engine.h"
#include <vector>
#include <memory>

// ============================================
// Collision Body Types Found in Binary
// ============================================

/**
 * CollisionBody - Base collision body class
 * Found in binary as "CollisionBody"
 */
class CollisionBody {
public:
    enum class Type {
        Dynamic,
        Static,
        TrackPolygon,
        Car,
        WorldObject
    };
    
    CollisionBody(Type type);
    virtual ~CollisionBody();
    
    Type GetType() const { return m_type; }
    int GetId() const { return m_id; }
    
    // Collision shape
    virtual bool Intersects(const CollisionBody* other) const = 0;
    virtual void GetBounds(float& minX, float& minY, float& minZ,
                          float& maxX, float& maxY, float& maxZ) const = 0;

protected:
    Type m_type;
    int m_id;
    static int s_nextId;
};

/**
 * CarCollisionBody
 * Collision body for vehicles
 * Found in binary as "CarCollisionBody"
 */
class CarCollisionBody : public CollisionBody {
public:
    CarCollisionBody();
    ~CarCollisionBody();
    
    void SetVehicle(Vehicle* vehicle) { m_vehicle = vehicle; }
    Vehicle* GetVehicle() const { return m_vehicle; }
    
    // Override collision methods
    bool Intersects(const CollisionBody* other) const override;
    void GetBounds(float& minX, float& minY, float& minZ,
                  float& maxX, float& maxY, float& maxZ) const override;

private:
    Vehicle* m_vehicle = nullptr;
    float m_radius = 2.0f;  // Approximate car radius
};

/**
 * TrackPolygonCollisionBody
 * Collision body for track polygons
 * Found in binary as "TrackPolygonCollisionBody"
 */
class TrackPolygonCollisionBody : public CollisionBody {
public:
    TrackPolygonCollisionBody();
    ~TrackPolygonCollisionBody();
    
    void SetPolygon(const std::vector<float>& vertices);
    const std::vector<float>& GetVertices() const { return m_vertices; }
    
    bool Intersects(const CollisionBody* other) const override;
    void GetBounds(float& minX, float& minY, float& minZ,
                  float& maxX, float& maxY, float& maxZ) const override;

private:
    std::vector<float> m_vertices;  // x, y, z, x, y, z, ...
};

/**
 * WorldObjectCollisionBody
 * Collision body for world objects
 * Found in binary as "WorldObjectCollisionBody"
 */
class WorldObjectCollisionBody : public CollisionBody {
public:
    WorldObjectCollisionBody();
    ~WorldObjectCollisionBody();
    
    void SetBounds(float minX, float minY, float minZ,
                   float maxX, float maxY, float maxZ);
    
    bool Intersects(const CollisionBody* other) const override;
    void GetBounds(float& minX, float& minY, float& minZ,
                  float& maxX, float& maxY, float& maxZ) const override;

private:
    float m_minX, m_minY, m_minZ;
    float m_maxX, m_maxY, m_maxZ;
};

/**
 * DynamicCollisionBody
 * Dynamic collision body
 * Found in binary as "DynamicCollisionBody"
 */
class DynamicCollisionBody : public CollisionBody {
public:
    DynamicCollisionBody();
    ~DynamicCollisionBody();
    
    void SetRigidBody(RigidBody* body) { m_rigidBody = body; }
    RigidBody* GetRigidBody() const { return m_rigidBody; }
    
    bool Intersects(const CollisionBody* other) const override;
    void GetBounds(float& minX, float& minY, float& minZ,
                  float& maxX, float& maxY, float& maxZ) const override;

private:
    RigidBody* m_rigidBody = nullptr;
};

// ============================================
// Enhanced PhysicsEngine with Collision Bodies
// ============================================

/**
 * Enhanced PhysicsEngine with collision body management
 */
class PhysicsEngineEnhanced : public PhysicsEngine {
public:
    PhysicsEngineEnhanced();
    ~PhysicsEngineEnhanced();
    
    // Collision body management
    CarCollisionBody* CreateCarCollisionBody(Vehicle* vehicle);
    TrackPolygonCollisionBody* CreateTrackPolygonCollisionBody();
    WorldObjectCollisionBody* CreateWorldObjectCollisionBody();
    DynamicCollisionBody* CreateDynamicCollisionBody(RigidBody* body);
    
    void RemoveCollisionBody(CollisionBody* body);
    
    // Collision detection with bodies
    void DetectCollisions() override;
    
    // Get all collision bodies
    const std::vector<std::unique_ptr<CollisionBody>>& GetAllCollisionBodies() const {
        return m_collisionBodies;
    }

private:
    std::vector<std::unique_ptr<CollisionBody>> m_collisionBodies;
    std::vector<std::pair<CollisionBody*, CollisionBody*>> m_collisionPairs;
};

// ============================================
// Implementation
// ============================================

// CollisionBody
int CollisionBody::s_nextId = 0;

inline CollisionBody::CollisionBody(Type type) 
    : m_type(type)
    , m_id(s_nextId++)
{
}

inline CollisionBody::~CollisionBody() {
}

// CarCollisionBody
inline CarCollisionBody::CarCollisionBody() 
    : CollisionBody(Type::Car)
{
}

inline CarCollisionBody::~CarCollisionBody() {
}

inline bool CarCollisionBody::Intersects(const CollisionBody* other) const {
    if (!other) return false;
    
    // Simple sphere-sphere or sphere-AABB
    float minX1, minY1, minZ1, maxX1, maxY1, maxZ1;
    GetBounds(minX1, minY1, minZ1, maxX1, maxY1, maxZ1);
    
    float minX2, minY2, minZ2, maxX2, maxY2, maxZ2;
    other->GetBounds(minX2, minY2, minZ2, maxX2, maxY2, maxZ2);
    
    return !(maxX1 < minX2 || minX1 > maxX2 ||
             maxY1 < minY2 || minY1 > maxY2 ||
             maxZ1 < minZ2 || minZ1 > maxZ2);
}

inline void CarCollisionBody::GetBounds(float& minX, float& minY, float& minZ,
                                       float& maxX, float& maxY, float& maxZ) const {
    if (m_vehicle) {
        // Get vehicle position
        float x = 0.0f, y = 0.0f, z = 0.0f;
        // vehicle->GetPosition(x, y, z);
        
        minX = x - m_radius;
        minY = y - m_radius;
        minZ = z - m_radius;
        maxX = x + m_radius;
        maxY = y + m_radius;
        maxZ = z + m_radius;
    } else {
        minX = minY = minZ = maxX = maxY = maxZ = 0.0f;
    }
}

// TrackPolygonCollisionBody
inline TrackPolygonCollisionBody::TrackPolygonCollisionBody() 
    : CollisionBody(Type::TrackPolygon)
{
}

inline TrackPolygonCollisionBody::~TrackPolygonCollisionBody() {
}

inline void TrackPolygonCollisionBody::SetPolygon(const std::vector<float>& vertices) {
    m_vertices = vertices;
}

inline bool TrackPolygonCollisionBody::Intersects(const CollisionBody* other) const {
    // Polygon-polygon or polygon-sphere intersection
    // Simplified for now
    return false;
}

inline void TrackPolygonCollisionBody::GetBounds(float& minX, float& minY, float& minZ,
                                                 float& maxX, float& maxY, float& maxZ) const {
    if (m_vertices.empty()) {
        minX = minY = minZ = maxX = maxY = maxZ = 0.0f;
        return;
    }
    
    minX = maxX = m_vertices[0];
    minY = maxY = m_vertices[1];
    minZ = maxZ = m_vertices[2];
    
    for (size_t i = 3; i < m_vertices.size(); i += 3) {
        float x = m_vertices[i];
        float y = m_vertices[i + 1];
        float z = m_vertices[i + 2];
        
        if (x < minX) minX = x;
        if (x > maxX) maxX = x;
        if (y < minY) minY = y;
        if (y > maxY) maxY = y;
        if (z < minZ) minZ = z;
        if (z > maxZ) maxZ = z;
    }
}

// WorldObjectCollisionBody
inline WorldObjectCollisionBody::WorldObjectCollisionBody() 
    : CollisionBody(Type::WorldObject)
    , m_minX(0.0f), m_minY(0.0f), m_minZ(0.0f)
    , m_maxX(0.0f), m_maxY(0.0f), m_maxZ(0.0f)
{
}

inline WorldObjectCollisionBody::~WorldObjectCollisionBody() {
}

inline void WorldObjectCollisionBody::SetBounds(float minX, float minY, float minZ,
                                                float maxX, float maxY, float maxZ) {
    m_minX = minX; m_minY = minY; m_minZ = minZ;
    m_maxX = maxX; m_maxY = maxY; m_maxZ = maxZ;
}

inline bool WorldObjectCollisionBody::Intersects(const CollisionBody* other) const {
    if (!other) return false;
    
    float minX2, minY2, minZ2, maxX2, maxY2, maxZ2;
    other->GetBounds(minX2, minY2, minZ2, maxX2, maxY2, maxZ2);
    
    return !(m_maxX < minX2 || m_minX > maxX2 ||
             m_maxY < minY2 || m_minY > maxY2 ||
             m_maxZ < minZ2 || m_minZ > maxZ2);
}

inline void WorldObjectCollisionBody::GetBounds(float& minX, float& minY, float& minZ,
                                                float& maxX, float& maxY, float& maxZ) const {
    minX = m_minX; minY = m_minY; minZ = m_minZ;
    maxX = m_maxX; maxY = m_maxY; maxZ = m_maxZ;
}

// DynamicCollisionBody
inline DynamicCollisionBody::DynamicCollisionBody() 
    : CollisionBody(Type::Dynamic)
{
}

inline DynamicCollisionBody::~DynamicCollisionBody() {
}

inline bool DynamicCollisionBody::Intersects(const CollisionBody* other) const {
    if (!other || !m_rigidBody) return false;
    
    float minX1, minY1, minZ1, maxX1, maxY1, maxZ1;
    GetBounds(minX1, minY1, minZ1, maxX1, maxY1, maxZ1);
    
    float minX2, minY2, minZ2, maxX2, maxY2, maxZ2;
    other->GetBounds(minX2, minY2, minZ2, maxX2, maxY2, maxZ2);
    
    return !(maxX1 < minX2 || minX1 > maxX2 ||
             maxY1 < minY2 || minY1 > maxY2 ||
             maxZ1 < minZ2 || minZ1 > maxZ2);
}

inline void DynamicCollisionBody::GetBounds(float& minX, float& minY, float& minZ,
                                           float& maxX, float& maxY, float& maxZ) const {
    if (m_rigidBody) {
        float x, y, z;
        m_rigidBody->GetPosition(x, y, z);
        
        // Assume radius of 1.0
        float radius = 1.0f;
        minX = x - radius;
        minY = y - radius;
        minZ = z - radius;
        maxX = x + radius;
        maxY = y + radius;
        maxZ = z + radius;
    } else {
        minX = minY = minZ = maxX = maxY = maxZ = 0.0f;
    }
}

// PhysicsEngineEnhanced
inline PhysicsEngineEnhanced::PhysicsEngineEnhanced() {
}

inline PhysicsEngineEnhanced::~PhysicsEngineEnhanced() {
    m_collisionBodies.clear();
}

inline CarCollisionBody* PhysicsEngineEnhanced::CreateCarCollisionBody(Vehicle* vehicle) {
    auto body = std::make_unique<CarCollisionBody>();
    body->SetVehicle(vehicle);
    CarCollisionBody* ptr = body.get();
    m_collisionBodies.push_back(std::move(body));
    return ptr;
}

inline TrackPolygonCollisionBody* PhysicsEngineEnhanced::CreateTrackPolygonCollisionBody() {
    auto body = std::make_unique<TrackPolygonCollisionBody>();
    TrackPolygonCollisionBody* ptr = body.get();
    m_collisionBodies.push_back(std::move(body));
    return ptr;
}

inline WorldObjectCollisionBody* PhysicsEngineEnhanced::CreateWorldObjectCollisionBody() {
    auto body = std::make_unique<WorldObjectCollisionBody>();
    WorldObjectCollisionBody* ptr = body.get();
    m_collisionBodies.push_back(std::move(body));
    return ptr;
}

inline DynamicCollisionBody* PhysicsEngineEnhanced::CreateDynamicCollisionBody(RigidBody* body) {
    auto collisionBody = std::make_unique<DynamicCollisionBody>();
    collisionBody->SetRigidBody(body);
    DynamicCollisionBody* ptr = collisionBody.get();
    m_collisionBodies.push_back(std::move(collisionBody));
    return ptr;
}

inline void PhysicsEngineEnhanced::RemoveCollisionBody(CollisionBody* body) {
    if (!body) return;
    
    m_collisionBodies.erase(
        std::remove_if(m_collisionBodies.begin(), m_collisionBodies.end(),
            [body](const std::unique_ptr<CollisionBody>& b) {
                return b.get() == body;
            }),
        m_collisionBodies.end()
    );
}

inline void PhysicsEngineEnhanced::DetectCollisions() {
    // Call base implementation
    PhysicsEngine::DetectCollisions();
    
    // Also detect collisions using collision bodies
    m_collisionPairs.clear();
    
    for (size_t i = 0; i < m_collisionBodies.size(); ++i) {
        for (size_t j = i + 1; j < m_collisionBodies.size(); ++j) {
            CollisionBody* body1 = m_collisionBodies[i].get();
            CollisionBody* body2 = m_collisionBodies[j].get();
            
            if (body1 && body2 && body1->Intersects(body2)) {
                m_collisionPairs.push_back({body1, body2});
            }
        }
    }
}

