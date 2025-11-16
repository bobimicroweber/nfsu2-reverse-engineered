// Lap Class - Detailed Implementation
// Extracted from SPEED2.EXE
// Lap tracking and timing system

#pragma once

#include <windows.h>
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>

// Forward declarations
class Vehicle;
class Checkpoint;

/**
 * Lap - Represents a single lap in a race
 * 
 * Found in binary:
 * - LAP_SNAPSHOT at offset 0x39544C
 * - CurrentLapTime at offset 0x3909B4
 * - BestLapTime at offset 0x397AC2
 * - LapNumber at offset 0x395440
 * - HUD_FEATURE_LAP_NUM, HUD_FEATURE_BEST_LAP, HUD_FEATURE_CURR_LAP
 * - POLapInfo, PPOLapInfo (lap info structures)
 * 
 * Provides:
 * - Lap timing and tracking
 * - Checkpoint management
 * - Lap snapshots for replay
 * - Best lap time tracking
 * - Sector times
 * - Lap statistics
 */
class Lap {
public:
    /**
     * Lap snapshot for replay/debugging
     */
    struct Snapshot {
        float timestamp;           // Time in lap
        float x, y, z;             // Position
        float rotationX, rotationY, rotationZ;  // Rotation
        float velocityX, velocityY, velocityZ; // Velocity
        float speed;               // Current speed
        float steering;             // Steering input
        float throttle;             // Throttle input
        float brake;                // Brake input
        int checkpointIndex;        // Current checkpoint
        float lapTime;              // Current lap time
    };
    
    /**
     * Sector time information
     */
    struct SectorTime {
        int sectorIndex;            // Sector number (0, 1, 2, etc.)
        float time;                 // Time for this sector
        bool isValid;               // Is time valid
    };
    
    Lap(int lapNumber);
    ~Lap();
    
    // ============================================
    // Initialization
    // ============================================
    
    /**
     * Initialize lap
     */
    void Initialize();
    
    /**
     * Start lap
     */
    void Start();
    
    /**
     * Finish lap
     */
    void Finish();
    
    /**
     * Reset lap
     */
    void Reset();
    
    // ============================================
    // Timing
    // ============================================
    
    /**
     * Update lap time
     */
    void Update(float deltaTime);
    
    /**
     * Get current lap time
     */
    float GetLapTime() const { return m_lapTime; }
    
    /**
     * Get lap time in formatted string (MM:SS.mmm)
     */
    std::string GetLapTimeString() const;
    
    /**
     * Get best lap time
     */
    float GetBestLapTime() const { return m_bestLapTime; }
    
    /**
     * Set best lap time
     */
    void SetBestLapTime(float time) { m_bestLapTime = time; }
    
    /**
     * Check if this is best lap
     */
    bool IsBestLap() const { return m_isBestLap; }
    
    /**
     * Get lap number
     */
    int GetLapNumber() const { return m_lapNumber; }
    
    /**
     * Check if lap is started
     */
    bool IsStarted() const { return m_started; }
    
    /**
     * Check if lap is finished
     */
    bool IsFinished() const { return m_finished; }
    
    /**
     * Check if lap is valid (all checkpoints passed)
     */
    bool IsValid() const { return m_valid; }
    
    /**
     * Set lap valid
     */
    void SetValid(bool valid) { m_valid = valid; }
    
    // ============================================
    // Checkpoints
    // ============================================
    
    /**
     * Get total checkpoints
     */
    int GetTotalCheckpoints() const { return m_totalCheckpoints; }
    
    /**
     * Set total checkpoints
     */
    void SetTotalCheckpoints(int count) { m_totalCheckpoints = count; }
    
    /**
     * Get current checkpoint index
     */
    int GetCurrentCheckpoint() const { return m_currentCheckpoint; }
    
    /**
     * Pass checkpoint
     */
    void PassCheckpoint(int checkpointIndex, float time);
    
    /**
     * Check if checkpoint passed
     */
    bool IsCheckpointPassed(int checkpointIndex) const;
    
    /**
     * Get checkpoint time
     */
    float GetCheckpointTime(int checkpointIndex) const;
    
    // ============================================
    // Sectors
    // ============================================
    
    /**
     * Get sector count
     */
    int GetSectorCount() const { return static_cast<int>(m_sectorTimes.size()); }
    
    /**
     * Get sector time
     */
    float GetSectorTime(int sectorIndex) const;
    
    /**
     * Get best sector time
     */
    float GetBestSectorTime(int sectorIndex) const;
    
    /**
     * Check if sector is best
     */
    bool IsBestSector(int sectorIndex) const;
    
    /**
     * Add sector time
     */
    void AddSectorTime(int sectorIndex, float time);
    
    // ============================================
    // Snapshots
    // ============================================
    
    /**
     * Take snapshot (LAP_SNAPSHOT)
     */
    void TakeSnapshot(Vehicle* vehicle);
    
    /**
     * Get snapshot at time
     */
    const Snapshot* GetSnapshotAtTime(float time) const;
    
    /**
     * Get all snapshots
     */
    const std::vector<Snapshot>& GetSnapshots() const { return m_snapshots; }
    
    /**
     * Clear snapshots
     */
    void ClearSnapshots() { m_snapshots.clear(); }
    
    /**
     * Get snapshot count
     */
    int GetSnapshotCount() const { return static_cast<int>(m_snapshots.size()); }
    
    // ============================================
    // Statistics
    // ============================================
    
    /**
     * Get average speed
     */
    float GetAverageSpeed() const { return m_averageSpeed; }
    
    /**
     * Get maximum speed
     */
    float GetMaximumSpeed() const { return m_maximumSpeed; }
    
    /**
     * Get minimum speed
     */
    float GetMinimumSpeed() const { return m_minimumSpeed; }
    
    /**
     * Get distance traveled
     */
    float GetDistance() const { return m_distance; }
    
    /**
     * Get number of checkpoints passed
     */
    int GetCheckpointsPassed() const { return m_checkpointsPassed; }

private:
    // Lap identification
    int m_lapNumber;                // Lap number (1, 2, 3, etc.)
    
    // Timing
    float m_lapTime;                // Current lap time
    float m_startTime;              // Start time (race time)
    float m_finishTime;             // Finish time (race time)
    float m_bestLapTime;            // Best lap time
    bool m_isBestLap;               // Is this the best lap
    bool m_started;                 // Lap started
    bool m_finished;                // Lap finished
    bool m_valid;                   // Lap is valid (all checkpoints)
    
    // Checkpoints
    int m_totalCheckpoints;         // Total checkpoints in lap
    int m_currentCheckpoint;        // Current checkpoint index
    int m_checkpointsPassed;        // Number of checkpoints passed
    std::vector<bool> m_checkpointPassed;      // Checkpoint passed flags
    std::vector<float> m_checkpointTimes;      // Time at each checkpoint
    
    // Sectors
    std::vector<SectorTime> m_sectorTimes;     // Sector times
    std::vector<float> m_bestSectorTimes;     // Best sector times
    
    // Snapshots
    std::vector<Snapshot> m_snapshots;        // Lap snapshots
    float m_snapshotInterval;                 // Time between snapshots
    float m_lastSnapshotTime;                 // Time of last snapshot
    
    // Statistics
    float m_averageSpeed;           // Average speed during lap
    float m_maximumSpeed;           // Maximum speed during lap
    float m_minimumSpeed;           // Minimum speed during lap
    float m_distance;               // Distance traveled
    float m_totalSpeed;             // Sum of speeds for average
    int m_speedSamples;             // Number of speed samples
    
    // Timing
    float m_totalTime;               // Total time elapsed
    
    // Constants
    static constexpr float SNAPSHOT_INTERVAL = 0.1f;  // Take snapshot every 0.1 seconds
    static constexpr int MAX_SNAPSHOTS = 1000;        // Maximum snapshots per lap
    
    // Internal methods
    void UpdateStatistics(float deltaTime, Vehicle* vehicle);
    void UpdateSnapshots(float deltaTime, Vehicle* vehicle);
    void CalculateSectors();
    bool ValidateLap();
    std::string FormatTime(float time) const;
};

// ============================================
// Lap Implementation
// ============================================

inline Lap::Lap(int lapNumber) : m_lapNumber(lapNumber) {
    m_lapTime = 0.0f;
    m_startTime = 0.0f;
    m_finishTime = 0.0f;
    m_bestLapTime = 999999.0f;
    m_isBestLap = false;
    m_started = false;
    m_finished = false;
    m_valid = false;
    m_totalCheckpoints = 0;
    m_currentCheckpoint = 0;
    m_checkpointsPassed = 0;
    m_snapshotInterval = SNAPSHOT_INTERVAL;
    m_lastSnapshotTime = 0.0f;
    m_averageSpeed = 0.0f;
    m_maximumSpeed = 0.0f;
    m_minimumSpeed = 999999.0f;
    m_distance = 0.0f;
    m_totalSpeed = 0.0f;
    m_speedSamples = 0;
    m_totalTime = 0.0f;
}

inline Lap::~Lap() {
    ClearSnapshots();
}

inline void Lap::Initialize() {
    m_lapTime = 0.0f;
    m_startTime = 0.0f;
    m_finishTime = 0.0f;
    m_isBestLap = false;
    m_started = false;
    m_finished = false;
    m_valid = false;
    m_currentCheckpoint = 0;
    m_checkpointsPassed = 0;
    m_checkpointPassed.clear();
    m_checkpointTimes.clear();
    m_checkpointPassed.resize(m_totalCheckpoints, false);
    m_checkpointTimes.resize(m_totalCheckpoints, 0.0f);
    m_sectorTimes.clear();
    m_snapshots.clear();
    m_lastSnapshotTime = 0.0f;
    m_averageSpeed = 0.0f;
    m_maximumSpeed = 0.0f;
    m_minimumSpeed = 999999.0f;
    m_distance = 0.0f;
    m_totalSpeed = 0.0f;
    m_speedSamples = 0;
    m_totalTime = 0.0f;
}

inline void Lap::Start() {
    m_started = true;
    m_finished = false;
    m_lapTime = 0.0f;
    m_currentCheckpoint = 0;
    m_checkpointsPassed = 0;
    m_lastSnapshotTime = 0.0f;
    m_totalTime = 0.0f;
    
    // Reset checkpoint flags
    for (int i = 0; i < m_checkpointPassed.size(); i++) {
        m_checkpointPassed[i] = false;
        m_checkpointTimes[i] = 0.0f;
    }
}

inline void Lap::Finish() {
    if (!m_started) return;
    
    m_finished = true;
    m_finishTime = m_totalTime;
    
    // Validate lap
    m_valid = ValidateLap();
    
    // Check if best lap
    if (m_valid && m_lapTime < m_bestLapTime) {
        m_bestLapTime = m_lapTime;
        m_isBestLap = true;
    } else {
        m_isBestLap = false;
    }
    
    // Calculate sectors
    CalculateSectors();
    
    // Calculate final statistics
    if (m_speedSamples > 0) {
        m_averageSpeed = m_totalSpeed / m_speedSamples;
    }
}

inline void Lap::Reset() {
    Initialize();
}

inline void Lap::Update(float deltaTime) {
    if (!m_started || m_finished) return;
    
    m_totalTime += deltaTime;
    m_lapTime += deltaTime;
}

inline void Lap::PassCheckpoint(int checkpointIndex, float time) {
    if (checkpointIndex < 0 || checkpointIndex >= m_totalCheckpoints) return;
    if (m_checkpointPassed[checkpointIndex]) return;
    
    m_checkpointPassed[checkpointIndex] = true;
    m_checkpointTimes[checkpointIndex] = time;
    m_currentCheckpoint = checkpointIndex;
    m_checkpointsPassed++;
}

inline bool Lap::IsCheckpointPassed(int checkpointIndex) const {
    if (checkpointIndex < 0 || checkpointIndex >= m_checkpointPassed.size()) return false;
    return m_checkpointPassed[checkpointIndex];
}

inline float Lap::GetCheckpointTime(int checkpointIndex) const {
    if (checkpointIndex < 0 || checkpointIndex >= m_checkpointTimes.size()) return 0.0f;
    return m_checkpointTimes[checkpointIndex];
}

inline float Lap::GetSectorTime(int sectorIndex) const {
    if (sectorIndex < 0 || sectorIndex >= m_sectorTimes.size()) return 0.0f;
    return m_sectorTimes[sectorIndex].time;
}

inline float Lap::GetBestSectorTime(int sectorIndex) const {
    if (sectorIndex < 0 || sectorIndex >= m_bestSectorTimes.size()) return 0.0f;
    return m_bestSectorTimes[sectorIndex];
}

inline bool Lap::IsBestSector(int sectorIndex) const {
    if (sectorIndex < 0 || sectorIndex >= m_sectorTimes.size()) return false;
    if (sectorIndex >= m_bestSectorTimes.size()) return true;
    return m_sectorTimes[sectorIndex].time < m_bestSectorTimes[sectorIndex];
}

inline void Lap::AddSectorTime(int sectorIndex, float time) {
    if (sectorIndex < 0) return;
    
    // Resize if needed
    if (sectorIndex >= m_sectorTimes.size()) {
        m_sectorTimes.resize(sectorIndex + 1);
        m_bestSectorTimes.resize(sectorIndex + 1, 999999.0f);
    }
    
    SectorTime sector;
    sector.sectorIndex = sectorIndex;
    sector.time = time;
    sector.isValid = true;
    
    m_sectorTimes[sectorIndex] = sector;
    
    // Update best sector time
    if (time < m_bestSectorTimes[sectorIndex]) {
        m_bestSectorTimes[sectorIndex] = time;
    }
}

inline void Lap::TakeSnapshot(Vehicle* vehicle) {
    if (!vehicle) return;
    if (m_snapshots.size() >= MAX_SNAPSHOTS) return;
    
    Snapshot snapshot;
    snapshot.timestamp = m_lapTime;
    
    vehicle->GetPosition(snapshot.x, snapshot.y, snapshot.z);
    vehicle->GetRotation(snapshot.rotationX, snapshot.rotationY, snapshot.rotationZ);
    vehicle->GetVelocity(snapshot.velocityX, snapshot.velocityY, snapshot.velocityZ);
    snapshot.speed = vehicle->GetSpeed();
    snapshot.checkpointIndex = m_currentCheckpoint;
    snapshot.lapTime = m_lapTime;
    
    // Get inputs from vehicle controller
    // snapshot.steering = vehicle->GetController()->GetSteering();
    // snapshot.throttle = vehicle->GetController()->GetThrottle();
    // snapshot.brake = vehicle->GetController()->GetBrake();
    
    m_snapshots.push_back(snapshot);
    m_lastSnapshotTime = m_lapTime;
}

inline const Lap::Snapshot* Lap::GetSnapshotAtTime(float time) const {
    // Find closest snapshot to time
    const Snapshot* closest = nullptr;
    float closestDiff = 999999.0f;
    
    for (const auto& snapshot : m_snapshots) {
        float diff = std::abs(snapshot.timestamp - time);
        if (diff < closestDiff) {
            closestDiff = diff;
            closest = &snapshot;
        }
    }
    
    return closest;
}

inline void Lap::UpdateStatistics(float deltaTime, Vehicle* vehicle) {
    if (!vehicle) return;
    
    float speed = vehicle->GetSpeed();
    
    // Update speed statistics
    m_totalSpeed += speed;
    m_speedSamples++;
    
    if (speed > m_maximumSpeed) {
        m_maximumSpeed = speed;
    }
    if (speed < m_minimumSpeed) {
        m_minimumSpeed = speed;
    }
    
    // Update distance
    m_distance += speed * deltaTime;
}

inline void Lap::UpdateSnapshots(float deltaTime, Vehicle* vehicle) {
    if (!vehicle) return;
    
    // Take snapshot at intervals
    if (m_lapTime - m_lastSnapshotTime >= m_snapshotInterval) {
        TakeSnapshot(vehicle);
    }
}

inline void Lap::CalculateSectors() {
    // Calculate sector times from checkpoints
    // Sectors are typically divided by checkpoints
    if (m_checkpointTimes.size() < 2) return;
    
    m_sectorTimes.clear();
    
    for (int i = 1; i < m_checkpointTimes.size(); i++) {
        float sectorTime = m_checkpointTimes[i] - m_checkpointTimes[i - 1];
        AddSectorTime(i - 1, sectorTime);
    }
}

inline bool Lap::ValidateLap() {
    // Lap is valid if all checkpoints are passed
    if (m_checkpointsPassed < m_totalCheckpoints) return false;
    
    // Check if checkpoints are passed in order
    for (int i = 0; i < m_totalCheckpoints; i++) {
        if (!m_checkpointPassed[i]) return false;
    }
    
    return true;
}

inline std::string Lap::FormatTime(float time) const {
    int minutes = static_cast<int>(time / 60.0f);
    float seconds = time - (minutes * 60.0f);
    int sec = static_cast<int>(seconds);
    int msec = static_cast<int>((seconds - sec) * 1000.0f);
    
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%02d:%02d.%03d", minutes, sec, msec);
    return std::string(buffer);
}

inline std::string Lap::GetLapTimeString() const {
    return FormatTime(m_lapTime);
}

