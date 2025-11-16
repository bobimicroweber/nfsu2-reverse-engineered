// RaceTrack Class - Detailed Implementation
// Extracted from SPEED2.EXE
// Race track management and navigation system

#pragma once

#include <windows.h>
#include <cmath>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include "checkpoint.h"

// Forward declarations
class Vehicle;
class Lap;

/**
 * RaceTrack - Represents a race track
 * 
 * Found in binary:
 * - TRACK, TrackMap, TRACKMAP strings
 * - TrackMapMemory, TrackMapTarget
 * - TrackDirection, TrackIcon
 * - TrackMode, ShortTrack, DragTrack, DriftTrack
 * - TRACKS\TRACKMAP%d.bin (track map files)
 * 
 * Provides:
 * - Checkpoint management
 * - Racing line calculation
 * - Track navigation
 * - Sector management
 * - Track metadata
 * - Start/finish line
 */
class RaceTrack {
public:
    /**
     * Track type
     */
    enum class Type {
        Circuit,     // Circuit track (looped)
        PointToPoint, // Point-to-point track
        Drag,        // Drag strip
        Drift,       // Drift track
        ShortTrack   // Short track
    };
    
    /**
     * Track direction
     */
    enum class Direction {
        Forward,     // Forward direction
        Reverse,     // Reverse direction
        Both         // Both directions
    };
    
    /**
     * Racing line point
     */
    struct RacingLinePoint {
        float x, y, z;              // Position
        float speed;                // Recommended speed
        float turnAngle;            // Turn angle at this point
        int checkpointIndex;        // Associated checkpoint
    };
    
    /**
     * Sector information
     */
    struct Sector {
        int sectorIndex;            // Sector number (0, 1, 2, etc.)
        int startCheckpoint;        // Starting checkpoint
        int endCheckpoint;          // Ending checkpoint
        float distance;             // Sector distance
        float bestTime;             // Best sector time
    };
    
    RaceTrack(const std::string& name);
    ~RaceTrack();
    
    // ============================================
    // Initialization
    // ============================================
    
    /**
     * Initialize track
     */
    bool Initialize();
    
    /**
     * Load track from file
     */
    bool LoadFromFile(const std::string& filename);
    
    /**
     * Save track to file
     */
    bool SaveToFile(const std::string& filename) const;
    
    /**
     * Shutdown track
     */
    void Shutdown();
    
    // ============================================
    // Properties
    // ============================================
    
    /**
     * Get track name
     */
    const std::string& GetName() const { return m_name; }
    
    /**
     * Set track name
     */
    void SetName(const std::string& name) { m_name = name; }
    
    /**
     * Get track type
     */
    Type GetType() const { return m_type; }
    
    /**
     * Set track type
     */
    void SetType(Type type) { m_type = type; }
    
    /**
     * Get track direction
     */
    Direction GetDirection() const { return m_direction; }
    
    /**
     * Set track direction
     */
    void SetDirection(Direction direction) { m_direction = direction; }
    
    /**
     * Get track length
     */
    float GetLength() const { return m_length; }
    
    /**
     * Set track length
     */
    void SetLength(float length) { m_length = length; }
    
    /**
     * Get number of laps
     */
    int GetLapCount() const { return m_lapCount; }
    
    /**
     * Set number of laps
     */
    void SetLapCount(int laps) { m_lapCount = laps; }
    
    // ============================================
    // Checkpoints
    // ============================================
    
    /**
     * Add checkpoint
     */
    void AddCheckpoint(Checkpoint* checkpoint);
    
    /**
     * Get checkpoint
     */
    Checkpoint* GetCheckpoint(int index);
    const Checkpoint* GetCheckpoint(int index) const;
    
    /**
     * Get checkpoint count
     */
    int GetCheckpointCount() const { return static_cast<int>(m_checkpoints.size()); }
    
    /**
     * Get start checkpoint (start/finish line)
     */
    Checkpoint* GetStartCheckpoint();
    const Checkpoint* GetStartCheckpoint() const;
    
    /**
     * Get checkpoint index
     */
    int GetCheckpointIndex(Checkpoint* checkpoint) const;
    
    /**
     * Get next checkpoint
     */
    Checkpoint* GetNextCheckpoint(int currentIndex);
    
    /**
     * Get previous checkpoint
     */
    Checkpoint* GetPreviousCheckpoint(int currentIndex);
    
    /**
     * Find nearest checkpoint to position
     */
    int FindNearestCheckpoint(float x, float y, float z) const;
    
    // ============================================
    // Racing Line
    // ============================================
    
    /**
     * Calculate racing line
     */
    void CalculateRacingLine();
    
    /**
     * Get racing line point at checkpoint
     */
    bool GetRacingLinePoint(int checkpointIndex, float& x, float& y, float& z) const;
    
    /**
     * Get racing line point at distance
     */
    bool GetRacingLinePointAtDistance(float distance, float& x, float& y, float& z) const;
    
    /**
     * Get racing line point count
     */
    int GetRacingLinePointCount() const { return static_cast<int>(m_racingLine.size()); }
    
    /**
     * Get all racing line points
     */
    const std::vector<RacingLinePoint>& GetRacingLine() const { return m_racingLine; }
    
    /**
     * Set racing line enabled
     */
    void SetRacingLineEnabled(bool enabled) { m_racingLineEnabled = enabled; }
    bool IsRacingLineEnabled() const { return m_racingLineEnabled; }
    
    // ============================================
    // Sectors
    // ============================================
    
    /**
     * Get sector count
     */
    int GetSectorCount() const { return static_cast<int>(m_sectors.size()); }
    
    /**
     * Get sector
     */
    const Sector* GetSector(int sectorIndex) const;
    
    /**
     * Add sector
     */
    void AddSector(int startCheckpoint, int endCheckpoint);
    
    /**
     * Calculate sectors from checkpoints
     */
    void CalculateSectors();
    
    /**
     * Get sector for checkpoint
     */
    int GetSectorForCheckpoint(int checkpointIndex) const;
    
    // ============================================
    // Navigation
    // ============================================
    
    /**
     * Get distance to next turn
     */
    float GetDistanceToNextTurn(float x, float y, float z, int checkpointIndex) const;
    
    /**
     * Get next turn direction (-1 = left, 1 = right, 0 = straight)
     */
    int GetNextTurnDirection(int checkpointIndex) const;
    
    /**
     * Get turn angle at checkpoint
     */
    float GetTurnAngle(int checkpointIndex) const;
    
    /**
     * Get track position (0.0 to 1.0 along track)
     */
    float GetTrackPosition(float x, float y, float z) const;
    
    /**
     * Get position on track at distance
     */
    void GetPositionAtDistance(float distance, float& x, float& y, float& z) const;
    
    // ============================================
    // Update
    // ============================================
    
    /**
     * Update track
     */
    void Update(float deltaTime);

private:
    // Identification
    std::string m_name;             // Track name
    Type m_type;                    // Track type
    Direction m_direction;          // Track direction
    
    // Properties
    float m_length;                 // Track length
    int m_lapCount;                  // Number of laps
    
    // Checkpoints
    std::vector<Checkpoint*> m_checkpoints;  // Checkpoints on track
    int m_startCheckpointIndex;     // Start/finish checkpoint index
    
    // Racing line
    std::vector<RacingLinePoint> m_racingLine;  // Racing line points
    bool m_racingLineEnabled;       // Racing line enabled
    bool m_racingLineCalculated;     // Racing line calculated
    
    // Sectors
    std::vector<Sector> m_sectors;   // Track sectors
    
    // State
    bool m_initialized;             // Track initialized
    bool m_loaded;                  // Track loaded from file
    
    // Constants
    static constexpr float DEFAULT_TRACK_LENGTH = 5000.0f;  // Default track length
    static constexpr int DEFAULT_LAP_COUNT = 3;            // Default lap count
    
    // Internal methods
    void CalculateTrackLength();
    void BuildCheckpointChain();
    float CalculateDistanceBetweenCheckpoints(int index1, int index2) const;
    float CalculateTurnAngle(int checkpointIndex) const;
    void GenerateRacingLinePoints();
};

// ============================================
// RaceTrack Implementation
// ============================================

inline RaceTrack::RaceTrack(const std::string& name) : m_name(name) {
    m_type = Type::Circuit;
    m_direction = Direction::Forward;
    m_length = DEFAULT_TRACK_LENGTH;
    m_lapCount = DEFAULT_LAP_COUNT;
    m_startCheckpointIndex = -1;
    m_racingLineEnabled = true;
    m_racingLineCalculated = false;
    m_initialized = false;
    m_loaded = false;
}

inline RaceTrack::~RaceTrack() {
    Shutdown();
}

inline bool RaceTrack::Initialize() {
    if (m_initialized) return true;
    
    // Build checkpoint chain
    BuildCheckpointChain();
    
    // Calculate track length
    CalculateTrackLength();
    
    // Calculate sectors
    CalculateSectors();
    
    // Calculate racing line
    if (m_racingLineEnabled) {
        CalculateRacingLine();
    }
    
    m_initialized = true;
    return true;
}

inline void RaceTrack::Shutdown() {
    // Don't delete checkpoints, they're managed elsewhere
    m_checkpoints.clear();
    m_racingLine.clear();
    m_sectors.clear();
    m_initialized = false;
    m_loaded = false;
}

inline void RaceTrack::Update(float deltaTime) {
    // Update track systems if needed
}

inline void RaceTrack::AddCheckpoint(Checkpoint* checkpoint) {
    if (!checkpoint) return;
    
    // Check if already added
    for (Checkpoint* cp : m_checkpoints) {
        if (cp == checkpoint) return;
    }
    
    m_checkpoints.push_back(checkpoint);
    
    // Set checkpoint index
    checkpoint->SetIndex(static_cast<int>(m_checkpoints.size() - 1));
    
    // If it's a start checkpoint, mark it
    if (checkpoint->GetType() == Checkpoint::Type::Start) {
        m_startCheckpointIndex = static_cast<int>(m_checkpoints.size() - 1);
    }
}

inline Checkpoint* RaceTrack::GetCheckpoint(int index) {
    if (index < 0 || index >= m_checkpoints.size()) return nullptr;
    return m_checkpoints[index];
}

inline const Checkpoint* RaceTrack::GetCheckpoint(int index) const {
    if (index < 0 || index >= m_checkpoints.size()) return nullptr;
    return m_checkpoints[index];
}

inline Checkpoint* RaceTrack::GetStartCheckpoint() {
    if (m_startCheckpointIndex < 0 || m_startCheckpointIndex >= m_checkpoints.size()) {
        // Find first checkpoint if start not set
        for (Checkpoint* cp : m_checkpoints) {
            if (cp && cp->GetType() == Checkpoint::Type::Start) {
                return cp;
            }
        }
        // Return first checkpoint if no start found
        if (!m_checkpoints.empty()) {
            return m_checkpoints[0];
        }
        return nullptr;
    }
    return m_checkpoints[m_startCheckpointIndex];
}

inline const Checkpoint* RaceTrack::GetStartCheckpoint() const {
    return const_cast<RaceTrack*>(this)->GetStartCheckpoint();
}

inline int RaceTrack::GetCheckpointIndex(Checkpoint* checkpoint) const {
    if (!checkpoint) return -1;
    
    for (size_t i = 0; i < m_checkpoints.size(); i++) {
        if (m_checkpoints[i] == checkpoint) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

inline Checkpoint* RaceTrack::GetNextCheckpoint(int currentIndex) {
    if (currentIndex < 0 || currentIndex >= m_checkpoints.size()) return nullptr;
    
    int nextIndex = currentIndex + 1;
    if (nextIndex >= m_checkpoints.size()) {
        // Loop back to start for circuit tracks
        if (m_type == Type::Circuit) {
            nextIndex = 0;
        } else {
            return nullptr;  // End of track for point-to-point
        }
    }
    
    return m_checkpoints[nextIndex];
}

inline Checkpoint* RaceTrack::GetPreviousCheckpoint(int currentIndex) {
    if (currentIndex < 0 || currentIndex >= m_checkpoints.size()) return nullptr;
    
    int prevIndex = currentIndex - 1;
    if (prevIndex < 0) {
        // Loop back to end for circuit tracks
        if (m_type == Type::Circuit) {
            prevIndex = static_cast<int>(m_checkpoints.size() - 1);
        } else {
            return nullptr;  // Start of track for point-to-point
        }
    }
    
    return m_checkpoints[prevIndex];
}

inline int RaceTrack::FindNearestCheckpoint(float x, float y, float z) const {
    int nearestIndex = -1;
    float nearestDistance = 999999.0f;
    
    for (size_t i = 0; i < m_checkpoints.size(); i++) {
        if (!m_checkpoints[i]) continue;
        
        float distance = m_checkpoints[i]->GetDistanceToPoint(x, y, z);
        if (distance < nearestDistance) {
            nearestDistance = distance;
            nearestIndex = static_cast<int>(i);
        }
    }
    
    return nearestIndex;
}

inline void RaceTrack::CalculateRacingLine() {
    if (m_racingLineCalculated) return;
    
    m_racingLine.clear();
    GenerateRacingLinePoints();
    m_racingLineCalculated = true;
}

inline bool RaceTrack::GetRacingLinePoint(int checkpointIndex, float& x, float& y, float& z) const {
    if (checkpointIndex < 0 || checkpointIndex >= m_racingLine.size()) return false;
    
    x = m_racingLine[checkpointIndex].x;
    y = m_racingLine[checkpointIndex].y;
    z = m_racingLine[checkpointIndex].z;
    return true;
}

inline bool RaceTrack::GetRacingLinePointAtDistance(float distance, float& x, float& y, float& z) const {
    if (m_racingLine.empty()) return false;
    
    // Find closest racing line point
    int nearestIndex = 0;
    float nearestDistance = 999999.0f;
    
    for (size_t i = 0; i < m_racingLine.size(); i++) {
        float dx = m_racingLine[i].x - x;
        float dy = m_racingLine[i].y - y;
        float dz = m_racingLine[i].z - z;
        float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        
        if (dist < nearestDistance) {
            nearestDistance = dist;
            nearestIndex = static_cast<int>(i);
        }
    }
    
    if (nearestIndex < m_racingLine.size()) {
        x = m_racingLine[nearestIndex].x;
        y = m_racingLine[nearestIndex].y;
        z = m_racingLine[nearestIndex].z;
        return true;
    }
    
    return false;
}

inline const RaceTrack::Sector* RaceTrack::GetSector(int sectorIndex) const {
    if (sectorIndex < 0 || sectorIndex >= m_sectors.size()) return nullptr;
    return &m_sectors[sectorIndex];
}

inline void RaceTrack::AddSector(int startCheckpoint, int endCheckpoint) {
    Sector sector;
    sector.sectorIndex = static_cast<int>(m_sectors.size());
    sector.startCheckpoint = startCheckpoint;
    sector.endCheckpoint = endCheckpoint;
    sector.distance = CalculateDistanceBetweenCheckpoints(startCheckpoint, endCheckpoint);
    sector.bestTime = 999999.0f;
    
    m_sectors.push_back(sector);
}

inline void RaceTrack::CalculateSectors() {
    m_sectors.clear();
    
    if (m_checkpoints.size() < 2) return;
    
    // Divide track into sectors based on checkpoints
    int sectorsPerLap = 3;  // Typically 3 sectors per lap
    int checkpointsPerSector = static_cast<int>(m_checkpoints.size()) / sectorsPerLap;
    
    for (int i = 0; i < sectorsPerLap; i++) {
        int startCheckpoint = i * checkpointsPerSector;
        int endCheckpoint = (i + 1) * checkpointsPerSector;
        
        if (endCheckpoint >= m_checkpoints.size()) {
            endCheckpoint = static_cast<int>(m_checkpoints.size() - 1);
        }
        
        AddSector(startCheckpoint, endCheckpoint);
    }
}

inline int RaceTrack::GetSectorForCheckpoint(int checkpointIndex) const {
    for (const auto& sector : m_sectors) {
        if (checkpointIndex >= sector.startCheckpoint && checkpointIndex <= sector.endCheckpoint) {
            return sector.sectorIndex;
        }
    }
    return -1;
}

inline float RaceTrack::GetDistanceToNextTurn(float x, float y, float z, int checkpointIndex) const {
    // Find next checkpoint with turn
    for (int i = checkpointIndex + 1; i < m_checkpoints.size(); i++) {
        float turnAngle = GetTurnAngle(i);
        if (std::abs(turnAngle) > 0.1f) {
            // Found a turn, calculate distance
            Checkpoint* cp = GetCheckpoint(i);
            if (cp) {
                return cp->GetDistanceToPoint(x, y, z);
            }
        }
    }
    return 999999.0f;
}

inline int RaceTrack::GetNextTurnDirection(int checkpointIndex) const {
    if (checkpointIndex < 0 || checkpointIndex >= m_checkpoints.size()) return 0;
    
    float turnAngle = GetTurnAngle(checkpointIndex);
    if (turnAngle > 0.1f) return 1;  // Right turn
    if (turnAngle < -0.1f) return -1;  // Left turn
    return 0;  // Straight
}

inline float RaceTrack::GetTurnAngle(int checkpointIndex) const {
    return CalculateTurnAngle(checkpointIndex);
}

inline float RaceTrack::GetTrackPosition(float x, float y, float z) const {
    // Find nearest checkpoint
    int nearestIndex = FindNearestCheckpoint(x, y, z);
    if (nearestIndex < 0) return 0.0f;
    
    // Calculate position along track (0.0 to 1.0)
    return static_cast<float>(nearestIndex) / static_cast<float>(m_checkpoints.size());
}

inline void RaceTrack::GetPositionAtDistance(float distance, float& x, float& y, float& z) const {
    // Find checkpoint at distance
    float currentDistance = 0.0f;
    
    for (size_t i = 0; i < m_checkpoints.size(); i++) {
        Checkpoint* cp = m_checkpoints[i];
        if (!cp) continue;
        
        float cpX, cpY, cpZ;
        cp->GetPosition(cpX, cpY, cpZ);
        
        float segmentDistance = 0.0f;
        if (i > 0) {
            Checkpoint* prevCp = m_checkpoints[i - 1];
            if (prevCp) {
                float prevX, prevY, prevZ;
                prevCp->GetPosition(prevX, prevY, prevZ);
                segmentDistance = std::sqrt(
                    (cpX - prevX) * (cpX - prevX) +
                    (cpY - prevY) * (cpY - prevY) +
                    (cpZ - prevZ) * (cpZ - prevZ)
                );
            }
        }
        
        if (currentDistance + segmentDistance >= distance) {
            // Interpolate between checkpoints
            float t = (distance - currentDistance) / segmentDistance;
            if (i > 0) {
                Checkpoint* prevCp = m_checkpoints[i - 1];
                if (prevCp) {
                    float prevX, prevY, prevZ;
                    prevCp->GetPosition(prevX, prevY, prevZ);
                    x = prevX + (cpX - prevX) * t;
                    y = prevY + (cpY - prevY) * t;
                    z = prevZ + (cpZ - prevZ) * t;
                    return;
                }
            }
            x = cpX;
            y = cpY;
            z = cpZ;
            return;
        }
        
        currentDistance += segmentDistance;
    }
    
    // Default to last checkpoint
    if (!m_checkpoints.empty() && m_checkpoints.back()) {
        m_checkpoints.back()->GetPosition(x, y, z);
    }
}

inline void RaceTrack::CalculateTrackLength() {
    m_length = 0.0f;
    
    for (size_t i = 1; i < m_checkpoints.size(); i++) {
        m_length += CalculateDistanceBetweenCheckpoints(static_cast<int>(i - 1), static_cast<int>(i));
    }
    
    // Add distance from last to first for circuit tracks
    if (m_type == Type::Circuit && m_checkpoints.size() > 1) {
        m_length += CalculateDistanceBetweenCheckpoints(
            static_cast<int>(m_checkpoints.size() - 1), 0
        );
    }
}

inline void RaceTrack::BuildCheckpointChain() {
    // Set next/previous checkpoint indices
    for (size_t i = 0; i < m_checkpoints.size(); i++) {
        if (!m_checkpoints[i]) continue;
        
        int nextIndex = static_cast<int>(i + 1);
        if (nextIndex >= m_checkpoints.size()) {
            if (m_type == Type::Circuit) {
                nextIndex = 0;
            } else {
                nextIndex = -1;
            }
        }
        
        int prevIndex = static_cast<int>(i - 1);
        if (prevIndex < 0) {
            if (m_type == Type::Circuit) {
                prevIndex = static_cast<int>(m_checkpoints.size() - 1);
            } else {
                prevIndex = -1;
            }
        }
        
        m_checkpoints[i]->SetNextCheckpointIndex(nextIndex);
        m_checkpoints[i]->SetPreviousCheckpointIndex(prevIndex);
    }
}

inline float RaceTrack::CalculateDistanceBetweenCheckpoints(int index1, int index2) const {
    if (index1 < 0 || index1 >= m_checkpoints.size()) return 0.0f;
    if (index2 < 0 || index2 >= m_checkpoints.size()) return 0.0f;
    
    Checkpoint* cp1 = m_checkpoints[index1];
    Checkpoint* cp2 = m_checkpoints[index2];
    
    if (!cp1 || !cp2) return 0.0f;
    
    float x1, y1, z1;
    float x2, y2, z2;
    cp1->GetPosition(x1, y1, z1);
    cp2->GetPosition(x2, y2, z2);
    
    float dx = x2 - x1;
    float dy = y2 - y1;
    float dz = z2 - z1;
    
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

inline float RaceTrack::CalculateTurnAngle(int checkpointIndex) const {
    if (checkpointIndex < 0 || checkpointIndex >= m_checkpoints.size()) return 0.0f;
    
    Checkpoint* current = m_checkpoints[checkpointIndex];
    Checkpoint* next = GetNextCheckpoint(checkpointIndex);
    Checkpoint* prev = GetPreviousCheckpoint(checkpointIndex);
    
    if (!current || !next || !prev) return 0.0f;
    
    float cx, cy, cz;
    float nx, ny, nz;
    float px, py, pz;
    
    current->GetPosition(cx, cy, cz);
    next->GetPosition(nx, ny, nz);
    prev->GetPosition(px, py, pz);
    
    // Calculate vectors
    float v1x = cx - px;
    float v1y = cy - py;
    float v1z = cz - pz;
    
    float v2x = nx - cx;
    float v2y = ny - cy;
    float v2z = nz - cz;
    
    // Normalize vectors
    float len1 = std::sqrt(v1x * v1x + v1y * v1y + v1z * v1z);
    float len2 = std::sqrt(v2x * v2x + v2y * v2y + v2z * v2z);
    
    if (len1 < 0.001f || len2 < 0.001f) return 0.0f;
    
    v1x /= len1;
    v1y /= len1;
    v1z /= len1;
    v2x /= len2;
    v2y /= len2;
    v2z /= len2;
    
    // Calculate angle (simplified 2D)
    float angle = std::atan2(v2z, v2x) - std::atan2(v1z, v1x);
    
    // Normalize to -PI to PI
    while (angle > 3.14159f) angle -= 2.0f * 3.14159f;
    while (angle < -3.14159f) angle += 2.0f * 3.14159f;
    
    return angle;
}

inline void RaceTrack::GenerateRacingLinePoints() {
    m_racingLine.clear();
    
    // Generate racing line points from checkpoints
    for (size_t i = 0; i < m_checkpoints.size(); i++) {
        Checkpoint* cp = m_checkpoints[i];
        if (!cp) continue;
        
        RacingLinePoint point;
        cp->GetPosition(point.x, point.y, point.z);
        point.checkpointIndex = static_cast<int>(i);
        point.speed = 200.0f;  // Default speed
        point.turnAngle = CalculateTurnAngle(static_cast<int>(i));
        
        // Adjust speed based on turn
        if (std::abs(point.turnAngle) > 0.3f) {
            point.speed *= 0.7f;  // Reduce speed for turns
        }
        
        m_racingLine.push_back(point);
    }
}

inline bool RaceTrack::LoadFromFile(const std::string& filename) {
    // Load track data from file
    // Format: TRACKS\TRACKMAP%d.bin
    // 
    // Expected file format (inferred from binary):
    // - Header: Track name, type, direction, length, lap count
    // - Checkpoint count
    // - Checkpoint data array:
    //   - Position (x, y, z)
    //   - Type (Standard, Start, Sector, Waypoint)
    //   - Shape (Box, Sphere, Plane, Cylinder)
    //   - Size/Radius
    //   - Next/Previous checkpoint indices
    // - Racing line points (optional)
    // - Sector definitions (optional)
    
    FILE* file = fopen(filename.c_str(), "rb");
    if (!file) {
        return false;
    }
    
    // Read header
    // Track name (assumed max 256 chars)
    char nameBuffer[256] = {0};
    if (fread(nameBuffer, 1, 256, file) != 256) {
        fclose(file);
        return false;
    }
    m_name = std::string(nameBuffer);
    
    // Read track type
    int typeInt = 0;
    if (fread(&typeInt, sizeof(int), 1, file) != 1) {
        fclose(file);
        return false;
    }
    m_type = static_cast<Type>(typeInt);
    
    // Read direction
    int directionInt = 0;
    if (fread(&directionInt, sizeof(int), 1, file) != 1) {
        fclose(file);
        return false;
    }
    m_direction = static_cast<Direction>(directionInt);
    
    // Read length
    if (fread(&m_length, sizeof(float), 1, file) != 1) {
        fclose(file);
        return false;
    }
    
    // Read lap count
    if (fread(&m_lapCount, sizeof(int), 1, file) != 1) {
        fclose(file);
        return false;
    }
    
    // Read checkpoint count
    int checkpointCount = 0;
    if (fread(&checkpointCount, sizeof(int), 1, file) != 1) {
        fclose(file);
        return false;
    }
    
    // Clear existing checkpoints
    m_checkpoints.clear();
    
    // Read checkpoints
    for (int i = 0; i < checkpointCount; i++) {
        // Create checkpoint
        Checkpoint* checkpoint = new Checkpoint(i);
        
        // Read position
        float x, y, z;
        if (fread(&x, sizeof(float), 1, file) != 1 ||
            fread(&y, sizeof(float), 1, file) != 1 ||
            fread(&z, sizeof(float), 1, file) != 1) {
            delete checkpoint;
            fclose(file);
            return false;
        }
        checkpoint->SetPosition(x, y, z);
        
        // Read type
        int checkpointType = 0;
        if (fread(&checkpointType, sizeof(int), 1, file) != 1) {
            delete checkpoint;
            fclose(file);
            return false;
        }
        checkpoint->SetType(static_cast<Checkpoint::Type>(checkpointType));
        
        // Read shape
        int shapeInt = 0;
        if (fread(&shapeInt, sizeof(int), 1, file) != 1) {
            delete checkpoint;
            fclose(file);
            return false;
        }
        checkpoint->SetShape(static_cast<Checkpoint::Shape>(shapeInt));
        
        // Read size (for box/plane)
        float width, height, depth;
        if (fread(&width, sizeof(float), 1, file) != 1 ||
            fread(&height, sizeof(float), 1, file) != 1 ||
            fread(&depth, sizeof(float), 1, file) != 1) {
            delete checkpoint;
            fclose(file);
            return false;
        }
        checkpoint->SetSize(width, height, depth);
        
        // Read radius (for sphere/cylinder)
        float radius = 0.0f;
        if (fread(&radius, sizeof(float), 1, file) != 1) {
            delete checkpoint;
            fclose(file);
            return false;
        }
        checkpoint->SetRadius(radius);
        
        // Read next/previous checkpoint indices
        int nextIndex = -1, prevIndex = -1;
        if (fread(&nextIndex, sizeof(int), 1, file) != 1 ||
            fread(&prevIndex, sizeof(int), 1, file) != 1) {
            delete checkpoint;
            fclose(file);
            return false;
        }
        checkpoint->SetNextCheckpointIndex(nextIndex);
        checkpoint->SetPreviousCheckpointIndex(prevIndex);
        
        // Check if start checkpoint
        if (checkpoint->GetType() == Checkpoint::Type::Start) {
            m_startCheckpointIndex = i;
        }
        
        checkpoint->Initialize();
        m_checkpoints.push_back(checkpoint);
    }
    
    // Read racing line points (optional)
    int racingLinePointCount = 0;
    if (fread(&racingLinePointCount, sizeof(int), 1, file) == 1 && racingLinePointCount > 0) {
        m_racingLine.clear();
        m_racingLine.reserve(racingLinePointCount);
        
        for (int i = 0; i < racingLinePointCount; i++) {
            RacingLinePoint point;
            if (fread(&point.x, sizeof(float), 1, file) != 1 ||
                fread(&point.y, sizeof(float), 1, file) != 1 ||
                fread(&point.z, sizeof(float), 1, file) != 1 ||
                fread(&point.speed, sizeof(float), 1, file) != 1 ||
                fread(&point.turnAngle, sizeof(float), 1, file) != 1 ||
                fread(&point.checkpointIndex, sizeof(int), 1, file) != 1) {
                break;
            }
            m_racingLine.push_back(point);
        }
        m_racingLineCalculated = true;
    }
    
    // Read sectors (optional)
    int sectorCount = 0;
    if (fread(&sectorCount, sizeof(int), 1, file) == 1 && sectorCount > 0) {
        m_sectors.clear();
        m_sectors.reserve(sectorCount);
        
        for (int i = 0; i < sectorCount; i++) {
            Sector sector;
            if (fread(&sector.sectorIndex, sizeof(int), 1, file) != 1 ||
                fread(&sector.startCheckpoint, sizeof(int), 1, file) != 1 ||
                fread(&sector.endCheckpoint, sizeof(int), 1, file) != 1 ||
                fread(&sector.distance, sizeof(float), 1, file) != 1 ||
                fread(&sector.bestTime, sizeof(float), 1, file) != 1) {
                break;
            }
            m_sectors.push_back(sector);
        }
    }
    
    fclose(file);
    
    // Build checkpoint chain
    BuildCheckpointChain();
    
    // Calculate track length if not set
    if (m_length <= 0.0f) {
        CalculateTrackLength();
    }
    
    // Calculate sectors if not loaded
    if (m_sectors.empty()) {
        CalculateSectors();
    }
    
    // Calculate racing line if not loaded
    if (m_racingLine.empty() && m_racingLineEnabled) {
        CalculateRacingLine();
    }
    
    m_loaded = true;
    m_initialized = true;
    return true;
}

inline bool RaceTrack::SaveToFile(const std::string& filename) const {
    // Save track data to file
    // Format: TRACKS\TRACKMAP%d.bin
    
    FILE* file = fopen(filename.c_str(), "wb");
    if (!file) {
        return false;
    }
    
    // Write header
    // Track name (256 chars, null-padded)
    char nameBuffer[256] = {0};
    strncpy(nameBuffer, m_name.c_str(), 255);
    if (fwrite(nameBuffer, 1, 256, file) != 256) {
        fclose(file);
        return false;
    }
    
    // Write track type
    int typeInt = static_cast<int>(m_type);
    if (fwrite(&typeInt, sizeof(int), 1, file) != 1) {
        fclose(file);
        return false;
    }
    
    // Write direction
    int directionInt = static_cast<int>(m_direction);
    if (fwrite(&directionInt, sizeof(int), 1, file) != 1) {
        fclose(file);
        return false;
    }
    
    // Write length
    if (fwrite(&m_length, sizeof(float), 1, file) != 1) {
        fclose(file);
        return false;
    }
    
    // Write lap count
    if (fwrite(&m_lapCount, sizeof(int), 1, file) != 1) {
        fclose(file);
        return false;
    }
    
    // Write checkpoint count
    int checkpointCount = static_cast<int>(m_checkpoints.size());
    if (fwrite(&checkpointCount, sizeof(int), 1, file) != 1) {
        fclose(file);
        return false;
    }
    
    // Write checkpoints
    for (Checkpoint* checkpoint : m_checkpoints) {
        if (!checkpoint) continue;
        
        // Write position
        float x, y, z;
        checkpoint->GetPosition(x, y, z);
        if (fwrite(&x, sizeof(float), 1, file) != 1 ||
            fwrite(&y, sizeof(float), 1, file) != 1 ||
            fwrite(&z, sizeof(float), 1, file) != 1) {
            fclose(file);
            return false;
        }
        
        // Write type
        int checkpointType = static_cast<int>(checkpoint->GetType());
        if (fwrite(&checkpointType, sizeof(int), 1, file) != 1) {
            fclose(file);
            return false;
        }
        
        // Write shape
        int shapeInt = static_cast<int>(checkpoint->GetShape());
        if (fwrite(&shapeInt, sizeof(int), 1, file) != 1) {
            fclose(file);
            return false;
        }
        
        // Write size
        float width, height, depth;
        checkpoint->GetSize(width, height, depth);
        if (fwrite(&width, sizeof(float), 1, file) != 1 ||
            fwrite(&height, sizeof(float), 1, file) != 1 ||
            fwrite(&depth, sizeof(float), 1, file) != 1) {
            fclose(file);
            return false;
        }
        
        // Write radius
        float radius = checkpoint->GetRadius();
        if (fwrite(&radius, sizeof(float), 1, file) != 1) {
            fclose(file);
            return false;
        }
        
        // Write next/previous checkpoint indices
        int nextIndex = checkpoint->GetNextCheckpointIndex();
        int prevIndex = checkpoint->GetPreviousCheckpointIndex();
        if (fwrite(&nextIndex, sizeof(int), 1, file) != 1 ||
            fwrite(&prevIndex, sizeof(int), 1, file) != 1) {
            fclose(file);
            return false;
        }
    }
    
    // Write racing line points
    int racingLinePointCount = static_cast<int>(m_racingLine.size());
    if (fwrite(&racingLinePointCount, sizeof(int), 1, file) != 1) {
        fclose(file);
        return false;
    }
    
    for (const auto& point : m_racingLine) {
        if (fwrite(&point.x, sizeof(float), 1, file) != 1 ||
            fwrite(&point.y, sizeof(float), 1, file) != 1 ||
            fwrite(&point.z, sizeof(float), 1, file) != 1 ||
            fwrite(&point.speed, sizeof(float), 1, file) != 1 ||
            fwrite(&point.turnAngle, sizeof(float), 1, file) != 1 ||
            fwrite(&point.checkpointIndex, sizeof(int), 1, file) != 1) {
            fclose(file);
            return false;
        }
    }
    
    // Write sectors
    int sectorCount = static_cast<int>(m_sectors.size());
    if (fwrite(&sectorCount, sizeof(int), 1, file) != 1) {
        fclose(file);
        return false;
    }
    
    for (const auto& sector : m_sectors) {
        if (fwrite(&sector.sectorIndex, sizeof(int), 1, file) != 1 ||
            fwrite(&sector.startCheckpoint, sizeof(int), 1, file) != 1 ||
            fwrite(&sector.endCheckpoint, sizeof(int), 1, file) != 1 ||
            fwrite(&sector.distance, sizeof(float), 1, file) != 1 ||
            fwrite(&sector.bestTime, sizeof(float), 1, file) != 1) {
            fclose(file);
            return false;
        }
    }
    
    fclose(file);
    return true;
}

