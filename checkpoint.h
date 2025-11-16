// Checkpoint Class - Detailed Implementation
// Extracted from SPEED2.EXE
// Checkpoint detection and validation system

#pragma once

#include <windows.h>
#include <cmath>
#include <string>
#include <vector>

// Forward declarations
class Vehicle;
class RaceTrack;

/**
 * Checkpoint - Represents a checkpoint on a race track
 * 
 * Checkpoints are used to:
 * - Validate lap completion
 * - Track race progress
 * - Calculate sector times
 * - Ensure proper track navigation
 * 
 * Provides:
 * - Position and bounds
 * - Collision detection
 * - Checkpoint passing validation
 * - Visual representation
 * - Order validation
 */
class Checkpoint {
public:
    /**
     * Checkpoint type
     */
    enum class Type {
        Standard,    // Standard checkpoint
        Start,       // Start/finish line
        Sector,      // Sector marker
        Waypoint     // Navigation waypoint
    };
    
    /**
     * Checkpoint shape
     */
    enum class Shape {
        Box,         // Box/cube shape
        Sphere,      // Sphere shape
        Plane,       // Plane/gate shape
        Cylinder     // Cylinder shape
    };
    
    Checkpoint(int index, Type type = Type::Standard);
    ~Checkpoint();
    
    // ============================================
    // Initialization
    // ============================================
    
    /**
     * Initialize checkpoint
     */
    void Initialize();
    
    /**
     * Shutdown checkpoint
     */
    void Shutdown();
    
    // ============================================
    // Position and Bounds
    // ============================================
    
    /**
     * Set position
     */
    void SetPosition(float x, float y, float z);
    void GetPosition(float& x, float& y, float& z) const;
    
    /**
     * Set size (for box/plane shapes)
     */
    void SetSize(float width, float height, float depth);
    void GetSize(float& width, float& height, float& depth) const;
    
    /**
     * Set radius (for sphere/cylinder shapes)
     */
    void SetRadius(float radius) { m_radius = radius; }
    float GetRadius() const { return m_radius; }
    
    /**
     * Set shape
     */
    void SetShape(Shape shape) { m_shape = shape; }
    Shape GetShape() const { return m_shape; }
    
    /**
     * Get bounding box
     */
    void GetBounds(float& minX, float& minY, float& minZ,
                   float& maxX, float& maxY, float& maxZ) const;
    
    // ============================================
    // Detection
    // ============================================
    
    /**
     * Check if vehicle passed checkpoint
     */
    bool IsVehiclePassed(Vehicle* vehicle) const;
    
    /**
     * Check if point is inside checkpoint
     */
    bool IsPointInside(float x, float y, float z) const;
    
    /**
     * Get distance to checkpoint
     */
    float GetDistanceToPoint(float x, float y, float z) const;
    
    /**
     * Get distance to vehicle
     */
    float GetDistanceToVehicle(Vehicle* vehicle) const;
    
    /**
     * Check if checkpoint is active
     */
    bool IsActive() const { return m_active; }
    
    /**
     * Set checkpoint active
     */
    void SetActive(bool active) { m_active = active; }
    
    // ============================================
    // Properties
    // ============================================
    
    /**
     * Get checkpoint index
     */
    int GetIndex() const { return m_index; }
    
    /**
     * Set checkpoint index
     */
    void SetIndex(int index) { m_index = index; }
    
    /**
     * Get checkpoint type
     */
    Type GetType() const { return m_type; }
    
    /**
     * Set checkpoint type
     */
    void SetType(Type type) { m_type = type; }
    
    /**
     * Get name
     */
    const std::string& GetName() const { return m_name; }
    
    /**
     * Set name
     */
    void SetName(const std::string& name) { m_name = name; }
    
    // ============================================
    // Order Validation
    // ============================================
    
    /**
     * Get next checkpoint index
     */
    int GetNextCheckpointIndex() const { return m_nextCheckpointIndex; }
    
    /**
     * Set next checkpoint index
     */
    void SetNextCheckpointIndex(int index) { m_nextCheckpointIndex = index; }
    
    /**
     * Get previous checkpoint index
     */
    int GetPreviousCheckpointIndex() const { return m_previousCheckpointIndex; }
    
    /**
     * Set previous checkpoint index
     */
    void SetPreviousCheckpointIndex(int index) { m_previousCheckpointIndex = index; }
    
    /**
     * Check if checkpoint is required before this one
     */
    bool IsRequiredCheckpointPassed(int checkpointIndex) const;
    
    // ============================================
    // Visual
    // ============================================
    
    /**
     * Set visible
     */
    void SetVisible(bool visible) { m_visible = visible; }
    bool IsVisible() const { return m_visible; }
    
    /**
     * Set model name
     */
    void SetModelName(const std::string& modelName) { m_modelName = modelName; }
    const std::string& GetModelName() const { return m_modelName; }
    
    /**
     * Set color (for visualization)
     */
    void SetColor(float r, float g, float b, float a = 1.0f);
    void GetColor(float& r, float& g, float& b, float& a) const;

private:
    // Identification
    int m_index;                    // Checkpoint index (0, 1, 2, etc.)
    Type m_type;                    // Checkpoint type
    std::string m_name;             // Checkpoint name
    
    // Position
    float m_x = 0.0f;
    float m_y = 0.0f;
    float m_z = 0.0f;
    
    // Size (for box/plane)
    float m_width = 5.0f;
    float m_height = 5.0f;
    float m_depth = 2.0f;
    
    // Radius (for sphere/cylinder)
    float m_radius = 2.5f;
    
    // Shape
    Shape m_shape = Shape::Plane;   // Default to plane (gate)
    
    // Order
    int m_nextCheckpointIndex = -1;      // Next checkpoint index
    int m_previousCheckpointIndex = -1;  // Previous checkpoint index
    std::vector<int> m_requiredCheckpoints;  // Required checkpoints before this
    
    // State
    bool m_active = true;           // Is checkpoint active
    bool m_visible = true;          // Is checkpoint visible
    
    // Visual
    std::string m_modelName;
    float m_colorR = 0.0f;
    float m_colorG = 1.0f;
    float m_colorB = 0.0f;
    float m_colorA = 1.0f;
    
    // Constants
    static constexpr float DEFAULT_WIDTH = 5.0f;
    static constexpr float DEFAULT_HEIGHT = 5.0f;
    static constexpr float DEFAULT_DEPTH = 2.0f;
    static constexpr float DEFAULT_RADIUS = 2.5f;
    
    // Internal methods
    bool IsPointInsideBox(float x, float y, float z) const;
    bool IsPointInsideSphere(float x, float y, float z) const;
    bool IsPointInsidePlane(float x, float y, float z) const;
    bool IsPointInsideCylinder(float x, float y, float z) const;
};

// ============================================
// Checkpoint Implementation
// ============================================

inline Checkpoint::Checkpoint(int index, Type type) : m_index(index), m_type(type) {
    m_x = 0.0f;
    m_y = 0.0f;
    m_z = 0.0f;
    m_width = DEFAULT_WIDTH;
    m_height = DEFAULT_HEIGHT;
    m_depth = DEFAULT_DEPTH;
    m_radius = DEFAULT_RADIUS;
    m_shape = Shape::Plane;
    m_nextCheckpointIndex = -1;
    m_previousCheckpointIndex = -1;
    m_active = true;
    m_visible = true;
    m_colorR = 0.0f;
    m_colorG = 1.0f;
    m_colorB = 0.0f;
    m_colorA = 1.0f;
    
    // Generate default name
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "Checkpoint_%d", index);
    m_name = std::string(buffer);
}

inline Checkpoint::~Checkpoint() {
    Shutdown();
}

inline void Checkpoint::Initialize() {
    m_active = true;
    m_visible = true;
}

inline void Checkpoint::Shutdown() {
    m_active = false;
}

inline void Checkpoint::SetPosition(float x, float y, float z) {
    m_x = x;
    m_y = y;
    m_z = z;
}

inline void Checkpoint::GetPosition(float& x, float& y, float& z) const {
    x = m_x;
    y = m_y;
    z = m_z;
}

inline void Checkpoint::SetSize(float width, float height, float depth) {
    m_width = width;
    m_height = height;
    m_depth = depth;
}

inline void Checkpoint::GetSize(float& width, float& height, float& depth) const {
    width = m_width;
    height = m_height;
    depth = m_depth;
}

inline void Checkpoint::GetBounds(float& minX, float& minY, float& minZ,
                                  float& maxX, float& maxY, float& maxZ) const {
    switch (m_shape) {
        case Shape::Box:
            minX = m_x - m_width / 2.0f;
            maxX = m_x + m_width / 2.0f;
            minY = m_y - m_height / 2.0f;
            maxY = m_y + m_height / 2.0f;
            minZ = m_z - m_depth / 2.0f;
            maxZ = m_z + m_depth / 2.0f;
            break;
            
        case Shape::Sphere:
            minX = m_x - m_radius;
            maxX = m_x + m_radius;
            minY = m_y - m_radius;
            maxY = m_y + m_radius;
            minZ = m_z - m_radius;
            maxZ = m_z + m_radius;
            break;
            
        case Shape::Plane:
            // Plane extends in X and Z, thin in Y
            minX = m_x - m_width / 2.0f;
            maxX = m_x + m_width / 2.0f;
            minY = m_y - m_height / 2.0f;
            maxY = m_y + m_height / 2.0f;
            minZ = m_z - m_depth / 2.0f;
            maxZ = m_z + m_depth / 2.0f;
            break;
            
        case Shape::Cylinder:
            minX = m_x - m_radius;
            maxX = m_x + m_radius;
            minY = m_y - m_height / 2.0f;
            maxY = m_y + m_height / 2.0f;
            minZ = m_z - m_radius;
            maxZ = m_z + m_radius;
            break;
    }
}

inline bool Checkpoint::IsVehiclePassed(Vehicle* vehicle) const {
    if (!vehicle || !m_active) return false;
    
    float vx, vy, vz;
    vehicle->GetPosition(vx, vy, vz);
    
    return IsPointInside(vx, vy, vz);
}

inline bool Checkpoint::IsPointInside(float x, float y, float z) const {
    if (!m_active) return false;
    
    switch (m_shape) {
        case Shape::Box:
            return IsPointInsideBox(x, y, z);
        case Shape::Sphere:
            return IsPointInsideSphere(x, y, z);
        case Shape::Plane:
            return IsPointInsidePlane(x, y, z);
        case Shape::Cylinder:
            return IsPointInsideCylinder(x, y, z);
        default:
            return false;
    }
}

inline bool Checkpoint::IsPointInsideBox(float x, float y, float z) const {
    float halfWidth = m_width / 2.0f;
    float halfHeight = m_height / 2.0f;
    float halfDepth = m_depth / 2.0f;
    
    return (x >= m_x - halfWidth && x <= m_x + halfWidth &&
            y >= m_y - halfHeight && y <= m_y + halfHeight &&
            z >= m_z - halfDepth && z <= m_z + halfDepth);
}

inline bool Checkpoint::IsPointInsideSphere(float x, float y, float z) const {
    float dx = x - m_x;
    float dy = y - m_y;
    float dz = z - m_z;
    float distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    return distance <= m_radius;
}

inline bool Checkpoint::IsPointInsidePlane(float x, float y, float z) const {
    // Plane checkpoint: check if point is within width/depth bounds
    // Height is typically ignored for plane checkpoints
    float halfWidth = m_width / 2.0f;
    float halfDepth = m_depth / 2.0f;
    
    return (x >= m_x - halfWidth && x <= m_x + halfWidth &&
            z >= m_z - halfDepth && z <= m_z + halfDepth);
}

inline bool Checkpoint::IsPointInsideCylinder(float x, float y, float z) const {
    float dx = x - m_x;
    float dz = z - m_z;
    float distance2D = std::sqrt(dx * dx + dz * dz);
    
    if (distance2D > m_radius) return false;
    
    float halfHeight = m_height / 2.0f;
    return (y >= m_y - halfHeight && y <= m_y + halfHeight);
}

inline float Checkpoint::GetDistanceToPoint(float x, float y, float z) const {
    float dx = x - m_x;
    float dy = y - m_y;
    float dz = z - m_z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

inline float Checkpoint::GetDistanceToVehicle(Vehicle* vehicle) const {
    if (!vehicle) return 999999.0f;
    
    float vx, vy, vz;
    vehicle->GetPosition(vx, vy, vz);
    
    return GetDistanceToPoint(vx, vy, vz);
}

inline bool Checkpoint::IsRequiredCheckpointPassed(int checkpointIndex) const {
    for (int req : m_requiredCheckpoints) {
        if (req == checkpointIndex) return true;
    }
    return false;
}

inline void Checkpoint::SetColor(float r, float g, float b, float a) {
    m_colorR = r;
    m_colorG = g;
    m_colorB = b;
    m_colorA = a;
}

inline void Checkpoint::GetColor(float& r, float& g, float& b, float& a) const {
    r = m_colorR;
    g = m_colorG;
    b = m_colorB;
    a = m_colorA;
}

