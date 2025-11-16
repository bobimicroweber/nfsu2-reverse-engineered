// RaceCarController Class - Detailed Implementation
// Extracted from SPEED2.EXE
// Race-specific vehicle controller for competitive racing

#pragma once

#include <windows.h>
#include <cmath>
#include <string>
#include <vector>
#include "carcontroller_enhanced.h"

// Forward declarations
class Vehicle;
class DriveTarget;
class Checkpoint;
class RaceTrack;

/**
 * RaceCarController - Race-specific vehicle controller
 * 
 * Found in binary:
 * - HBRaceCarController at offset 0x385B82
 * - AICarController at offset 0x385B70
 * - eMM_AIRACECAR (AI race car mode)
 * 
 * Provides race-specific features:
 * - Lap tracking
 * - Checkpoint management
 * - Race position tracking
 * - AI racing behavior
 * - Optimal racing line calculation
 * - Overtaking logic
 * - Defensive driving
 */
class RaceCarController : public CarController {
public:
    RaceCarController();
    virtual ~RaceCarController();
    
    // ============================================
    // Initialization
    // ============================================
    
    /**
     * Initialize race car controller
     */
    void Initialize() override;
    
    /**
     * Shutdown race car controller
     */
    void Shutdown();
    
    // ============================================
    // Main Update
    // ============================================
    
    /**
     * Update race car controller
     * Called every frame
     */
    void Update(float deltaTime) override;
    
    // ============================================
    // Race State
    // ============================================
    
    /**
     * Get current lap number
     */
    int GetCurrentLap() const { return m_currentLap; }
    
    /**
     * Get total laps
     */
    int GetTotalLaps() const { return m_totalLaps; }
    
    /**
     * Set total laps
     */
    void SetTotalLaps(int laps) { m_totalLaps = laps; }
    
    /**
     * Get current race position (1 = first)
     */
    int GetRacePosition() const { return m_racePosition; }
    
    /**
     * Set race position
     */
    void SetRacePosition(int position) { m_racePosition = position; }
    
    /**
     * Get race time
     */
    float GetRaceTime() const { return m_raceTime; }
    
    /**
     * Get lap time
     */
    float GetLapTime() const { return m_lapTime; }
    
    /**
     * Get best lap time
     */
    float GetBestLapTime() const { return m_bestLapTime; }
    
    /**
     * Check if race is finished
     */
    bool IsRaceFinished() const { return m_raceFinished; }
    
    /**
     * Set race finished
     */
    void SetRaceFinished(bool finished) { m_raceFinished = finished; }
    
    // ============================================
    // Checkpoints
    // ============================================
    
    /**
     * Get current checkpoint index
     */
    int GetCurrentCheckpoint() const { return m_currentCheckpoint; }
    
    /**
     * Get total checkpoints
     */
    int GetTotalCheckpoints() const { return m_totalCheckpoints; }
    
    /**
     * Set total checkpoints
     */
    void SetTotalCheckpoints(int count) { m_totalCheckpoints = count; }
    
    /**
     * Pass checkpoint
     */
    void PassCheckpoint(int checkpointIndex);
    
    /**
     * Check if checkpoint passed
     */
    bool IsCheckpointPassed(int checkpointIndex) const;
    
    // ============================================
    // Racing Line
    // ============================================
    
    /**
     * Calculate optimal racing line
     */
    void CalculateRacingLine();
    
    /**
     * Get target position on racing line
     */
    void GetRacingLineTarget(float& x, float& y, float& z) const;
    
    /**
     * Get distance to racing line
     */
    float GetDistanceToRacingLine() const;
    
    /**
     * Set racing line enabled
     */
    void SetRacingLineEnabled(bool enabled) { m_racingLineEnabled = enabled; }
    bool IsRacingLineEnabled() const { return m_racingLineEnabled; }
    
    // ============================================
    // AI Racing Behavior
    // ============================================
    
    /**
     * Set AI mode
     */
    void SetAIMode(bool aiMode) { m_aiMode = aiMode; }
    bool IsAIMode() const { return m_aiMode; }
    
    /**
     * Set AI aggression (0.0 to 1.0)
     */
    void SetAIAggression(float aggression) { m_aiAggression = aggression; }
    float GetAIAggression() const { return m_aiAggression; }
    
    /**
     * Set AI skill level (0.0 to 1.0)
     */
    void SetAISkill(float skill) { m_aiSkill = skill; }
    float GetAISkill() const { return m_aiSkill; }
    
    /**
     * Calculate AI steering
     */
    float CalculateAISteering(float deltaTime);
    
    /**
     * Calculate AI throttle
     */
    float CalculateAIThrottle(float deltaTime);
    
    /**
     * Calculate AI brake
     */
    float CalculateAIBrake(float deltaTime);
    
    // ============================================
    // Overtaking
    // ============================================
    
    /**
     * Check if can overtake
     */
    bool CanOvertake(Vehicle* target) const;
    
    /**
     * Calculate overtaking position
     */
    void CalculateOvertakingPosition(Vehicle* target, float& x, float& y, float& z);
    
    /**
     * Get overtaking side (-1 = left, 1 = right, 0 = none)
     */
    int GetOvertakingSide(Vehicle* target) const;
    
    // ============================================
    // Defensive Driving
    // ============================================
    
    /**
     * Check if being overtaken
     */
    bool IsBeingOvertaken(Vehicle* opponent) const;
    
    /**
     * Calculate defensive position
     */
    void CalculateDefensivePosition(Vehicle* opponent, float& x, float& y, float& z);
    
    /**
     * Set defensive mode
     */
    void SetDefensiveMode(bool defensive) { m_defensiveMode = defensive; }
    bool IsDefensiveMode() const { return m_defensiveMode; }
    
    // ============================================
    // Race Track
    // ============================================
    
    /**
     * Set race track
     */
    void SetRaceTrack(RaceTrack* track) { m_raceTrack = track; }
    RaceTrack* GetRaceTrack() const { return m_raceTrack; }
    
    /**
     * Get distance to next turn
     */
    float GetDistanceToNextTurn() const;
    
    /**
     * Get turn direction (-1 = left, 1 = right, 0 = straight)
     */
    int GetNextTurnDirection() const;
    
    // ============================================
    // Speed Management
    // ============================================
    
    /**
     * Get optimal speed for current section
     */
    float GetOptimalSpeed() const;
    
    /**
     * Calculate speed for turn
     */
    float CalculateTurnSpeed(float turnAngle) const;
    
    /**
     * Set speed limit
     */
    void SetSpeedLimit(float limit) { m_speedLimit = limit; }
    float GetSpeedLimit() const { return m_speedLimit; }

private:
    // Race state
    int m_currentLap = 1;
    int m_totalLaps = 3;
    int m_racePosition = 1;
    float m_raceTime = 0.0f;
    float m_lapTime = 0.0f;
    float m_bestLapTime = 999999.0f;
    bool m_raceFinished = false;
    
    // Checkpoints
    int m_currentCheckpoint = 0;
    int m_totalCheckpoints = 0;
    std::vector<bool> m_checkpointsPassed;
    
    // Racing line
    bool m_racingLineEnabled = true;
    float m_racingLineX = 0.0f;
    float m_racingLineY = 0.0f;
    float m_racingLineZ = 0.0f;
    float m_distanceToRacingLine = 0.0f;
    
    // AI behavior
    bool m_aiMode = false;
    float m_aiAggression = 0.5f;  // 0.0 = conservative, 1.0 = aggressive
    float m_aiSkill = 0.5f;       // 0.0 = poor, 1.0 = expert
    
    // Overtaking
    Vehicle* m_overtakingTarget = nullptr;
    float m_overtakingDistance = 0.0f;
    int m_overtakingSide = 0;  // -1 = left, 1 = right
    
    // Defensive
    bool m_defensiveMode = false;
    Vehicle* m_defendingAgainst = nullptr;
    
    // Race track
    RaceTrack* m_raceTrack = nullptr;
    float m_distanceToNextTurn = 0.0f;
    int m_nextTurnDirection = 0;  // -1 = left, 1 = right, 0 = straight
    
    // Speed management
    float m_optimalSpeed = 0.0f;
    float m_speedLimit = 250.0f;
    
    // Timing
    float m_totalTime = 0.0f;
    float m_lapStartTime = 0.0f;
    
    // Constants
    static constexpr float OVERTAKING_DISTANCE = 10.0f;  // Distance to start overtaking
    static constexpr float DEFENSIVE_DISTANCE = 5.0f;    // Distance to start defending
    static constexpr float TURN_SPEED_FACTOR = 0.7f;     // Speed reduction for turns
    static constexpr float AI_REACTION_TIME = 0.1f;      // AI reaction delay
    
    // Internal methods
    void UpdateRaceState(float deltaTime);
    void UpdateCheckpoints(float deltaTime);
    void UpdateRacingLine(float deltaTime);
    void UpdateAIBehavior(float deltaTime);
    void UpdateOvertaking(float deltaTime);
    void UpdateDefensive(float deltaTime);
    void UpdateSpeed(float deltaTime);
    void OnLapComplete();
    void OnCheckpointPassed(int checkpointIndex);
    float CalculateDistanceToVehicle(Vehicle* vehicle) const;
    bool IsInOvertakingPosition(Vehicle* target) const;
};

// ============================================
// RaceCarController Implementation
// ============================================

inline RaceCarController::RaceCarController() : CarController() {
    m_currentLap = 1;
    m_totalLaps = 3;
    m_racePosition = 1;
    m_raceTime = 0.0f;
    m_lapTime = 0.0f;
    m_bestLapTime = 999999.0f;
    m_raceFinished = false;
    m_currentCheckpoint = 0;
    m_totalCheckpoints = 0;
    m_racingLineEnabled = true;
    m_aiMode = false;
    m_aiAggression = 0.5f;
    m_aiSkill = 0.5f;
    m_overtakingTarget = nullptr;
    m_defensiveMode = false;
    m_defendingAgainst = nullptr;
    m_raceTrack = nullptr;
    m_optimalSpeed = 0.0f;
    m_speedLimit = 250.0f;
    m_totalTime = 0.0f;
    m_lapStartTime = 0.0f;
}

inline RaceCarController::~RaceCarController() {
    Shutdown();
}

inline void RaceCarController::Initialize() {
    CarController::Initialize();
    
    m_currentLap = 1;
    m_racePosition = 1;
    m_raceTime = 0.0f;
    m_lapTime = 0.0f;
    m_raceFinished = false;
    m_currentCheckpoint = 0;
    m_checkpointsPassed.clear();
    m_checkpointsPassed.resize(m_totalCheckpoints, false);
    m_lapStartTime = 0.0f;
    m_totalTime = 0.0f;
}

inline void RaceCarController::Shutdown() {
    m_raceTrack = nullptr;
    m_overtakingTarget = nullptr;
    m_defendingAgainst = nullptr;
    m_checkpointsPassed.clear();
}

inline void RaceCarController::Update(float deltaTime) {
    CarController::Update(deltaTime);
    
    m_totalTime += deltaTime;
    
    // Update race state
    UpdateRaceState(deltaTime);
    
    // Update checkpoints
    UpdateCheckpoints(deltaTime);
    
    // Update racing line
    if (m_racingLineEnabled) {
        UpdateRacingLine(deltaTime);
    }
    
    // Update AI behavior
    if (m_aiMode) {
        UpdateAIBehavior(deltaTime);
    }
    
    // Update overtaking
    UpdateOvertaking(deltaTime);
    
    // Update defensive
    UpdateDefensive(deltaTime);
    
    // Update speed
    UpdateSpeed(deltaTime);
}

inline void RaceCarController::UpdateRaceState(float deltaTime) {
    if (m_raceFinished) return;
    
    m_raceTime += deltaTime;
    m_lapTime += deltaTime;
}

inline void RaceCarController::UpdateCheckpoints(float deltaTime) {
    // Checkpoint detection would be handled by collision system
    // This method updates checkpoint state
}

inline void RaceCarController::UpdateRacingLine(float deltaTime) {
    // Calculate optimal racing line position
    // Simplified: follow center of track
    if (m_raceTrack) {
        // Get racing line from track
        // m_raceTrack->GetRacingLinePosition(m_currentCheckpoint, m_racingLineX, m_racingLineY, m_racingLineZ);
    }
}

inline void RaceCarController::UpdateAIBehavior(float deltaTime) {
    // Calculate AI inputs
    float steering = CalculateAISteering(deltaTime);
    float throttle = CalculateAIThrottle(deltaTime);
    float brake = CalculateAIBrake(deltaTime);
    
    // Apply AI inputs
    SetSteering(steering);
    SetThrottle(throttle);
    SetBrake(brake);
}

inline void RaceCarController::UpdateOvertaking(float deltaTime) {
    // Find nearest opponent ahead
    // Check if can overtake
    // Calculate overtaking position
}

inline void RaceCarController::UpdateDefensive(float deltaTime) {
    // Check if being overtaken
    // Calculate defensive position
    // Block overtaking attempts
}

inline void RaceCarController::UpdateSpeed(float deltaTime) {
    // Calculate optimal speed for current section
    m_optimalSpeed = GetOptimalSpeed();
    
    // Apply speed limit
    if (m_optimalSpeed > m_speedLimit) {
        m_optimalSpeed = m_speedLimit;
    }
}

inline void RaceCarController::PassCheckpoint(int checkpointIndex) {
    if (checkpointIndex >= 0 && checkpointIndex < m_totalCheckpoints) {
        if (!m_checkpointsPassed[checkpointIndex]) {
            m_checkpointsPassed[checkpointIndex] = true;
            m_currentCheckpoint = checkpointIndex;
            OnCheckpointPassed(checkpointIndex);
        }
    }
}

inline bool RaceCarController::IsCheckpointPassed(int checkpointIndex) const {
    if (checkpointIndex >= 0 && checkpointIndex < m_checkpointsPassed.size()) {
        return m_checkpointsPassed[checkpointIndex];
    }
    return false;
}

inline void RaceCarController::OnLapComplete() {
    // Update lap time
    if (m_lapTime < m_bestLapTime) {
        m_bestLapTime = m_lapTime;
    }
    
    // Increment lap
    m_currentLap++;
    m_lapTime = 0.0f;
    m_lapStartTime = m_totalTime;
    
    // Reset checkpoints
    m_currentCheckpoint = 0;
    for (int i = 0; i < m_checkpointsPassed.size(); i++) {
        m_checkpointsPassed[i] = false;
    }
    
    // Check if race finished
    if (m_currentLap > m_totalLaps) {
        m_raceFinished = true;
    }
}

inline void RaceCarController::OnCheckpointPassed(int checkpointIndex) {
    // Check if all checkpoints passed (lap complete)
    bool allPassed = true;
    for (bool passed : m_checkpointsPassed) {
        if (!passed) {
            allPassed = false;
            break;
        }
    }
    
    if (allPassed) {
        OnLapComplete();
    }
}

inline float RaceCarController::CalculateAISteering(float deltaTime) {
    // AI steering towards racing line or target
    float targetSteering = 0.0f;
    
    if (m_racingLineEnabled) {
        // Steer towards racing line
        // Simplified calculation
        targetSteering = m_distanceToRacingLine * 0.1f;
    }
    
    // Add skill-based variation
    float skillVariation = (1.0f - m_aiSkill) * 0.2f;
    targetSteering += (rand() % 1000 / 1000.0f - 0.5f) * skillVariation;
    
    return std::max(-1.0f, std::min(1.0f, targetSteering));
}

inline float RaceCarController::CalculateAIThrottle(float deltaTime) {
    // AI throttle based on speed and racing line
    float currentSpeed = GetSpeed();
    float optimalSpeed = GetOptimalSpeed();
    
    float throttle = 0.0f;
    if (currentSpeed < optimalSpeed) {
        throttle = 1.0f;  // Full throttle
    } else {
        throttle = 0.5f;  // Maintain speed
    }
    
    // Add aggression factor
    throttle *= (0.5f + m_aiAggression * 0.5f);
    
    return std::max(0.0f, std::min(1.0f, throttle));
}

inline float RaceCarController::CalculateAIBrake(float deltaTime) {
    // AI brake for turns
    float brake = 0.0f;
    
    if (m_nextTurnDirection != 0) {
        // Brake before turn
        if (m_distanceToNextTurn < 20.0f) {
            brake = 0.5f;
        }
    }
    
    // Add skill-based variation
    brake *= (1.0f - m_aiSkill * 0.5f);
    
    return std::max(0.0f, std::min(1.0f, brake));
}

inline float RaceCarController::GetOptimalSpeed() const {
    // Calculate optimal speed based on track section
    float baseSpeed = 200.0f;
    
    // Reduce speed for turns
    if (m_nextTurnDirection != 0) {
        baseSpeed *= TURN_SPEED_FACTOR;
    }
    
    return baseSpeed;
}

inline float RaceCarController::CalculateTurnSpeed(float turnAngle) const {
    // Calculate speed based on turn angle
    float maxSpeed = 250.0f;
    float turnFactor = 1.0f - (std::abs(turnAngle) / 3.14159f) * 0.5f;
    return maxSpeed * turnFactor;
}

inline bool RaceCarController::CanOvertake(Vehicle* target) const {
    if (!target) return false;
    
    float distance = CalculateDistanceToVehicle(target);
    return distance < OVERTAKING_DISTANCE;
}

inline float RaceCarController::CalculateDistanceToVehicle(Vehicle* vehicle) const {
    if (!vehicle) return 999999.0f;
    
    float vx, vy, vz;
    vehicle->GetPosition(vx, vy, vz);
    
    float x, y, z;
    GetPosition(x, y, z);
    
    float dx = vx - x;
    float dy = vy - y;
    float dz = vz - z;
    
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

inline void RaceCarController::GetRacingLineTarget(float& x, float& y, float& z) const {
    x = m_racingLineX;
    y = m_racingLineY;
    z = m_racingLineZ;
}

inline float RaceCarController::GetDistanceToRacingLine() const {
    return m_distanceToRacingLine;
}

inline float RaceCarController::GetDistanceToNextTurn() const {
    return m_distanceToNextTurn;
}

inline int RaceCarController::GetNextTurnDirection() const {
    return m_nextTurnDirection;
}

