// Enhanced CarController class structure
// Extracted and reverse-engineered from SPEED2.EXE
// Based on string analysis and binary structure

#pragma once

#include <cstdint>

// Forward declarations
class DriveTarget;
class DrivingChoice;
class PlayerSteering;
class RealDriver;
class DriftDriver;

/**
 * CarController - Main vehicle control class
 * 
 * This class handles vehicle physics, steering, acceleration, and braking.
 * Located at offset 0x38445C in SPEED2.EXE
 */
class CarController {
public:
    // ============================================
    // Construction/Destruction
    // ============================================
    CarController();
    virtual ~CarController();
    
    // ============================================
    // Virtual Methods (VTable)
    // ============================================
    // VTable located around 0x3843F0-0x384430
    
    // Method 0: Possibly Update() or Initialize()
    virtual void Update(float deltaTime);
    
    // Method 1: Possibly Reset() or Cleanup()
    virtual void Reset();
    
    // Method 2: Possibly GetType() or GetState()
    virtual int GetType() const;
    
    // ============================================
    // Steering Control
    // ============================================
    // Related to PlayerSteering at 0x3844C4
    
    /**
     * Set steering angle
     * @param angle Steering angle in radians (-1.0 to 1.0)
     */
    void SetSteering(float angle);
    
    /**
     * Get current steering angle
     */
    float GetSteering() const;
    
    /**
     * Apply steering input
     * @param input Steering input from player (-1.0 to 1.0)
     */
    void Steer(float input);
    
    // ============================================
    // Throttle/Brake Control
    // ============================================
    // Related to "Gas Brake" at 0x384224
    
    /**
     * Set throttle (gas) input
     * @param throttle Throttle value (0.0 to 1.0)
     */
    void SetThrottle(float throttle);
    
    /**
     * Set brake input
     * @param brake Brake value (0.0 to 1.0)
     */
    void SetBrake(float brake);
    
    /**
     * Get current throttle value
     */
    float GetThrottle() const;
    
    /**
     * Get current brake value
     */
    float GetBrake() const;
    
    /**
     * Apply emergency brake
     */
    void EmergencyBrake();
    
    // ============================================
    // Speed and Movement
    // ============================================
    
    /**
     * Get current speed
     * @return Speed in game units
     */
    float GetSpeed() const;
    
    /**
     * Get target speed (from DriveTarget)
     */
    float GetTargetSpeed() const;
    
    /**
     * Set target speed
     */
    void SetTargetSpeed(float speed);
    
    // ============================================
    // Driving Modes
    // ============================================
    // Related to DrivingChoice at 0x38447C
    
    /**
     * Set driving choice/mode
     */
    void SetDrivingChoice(int choice);
    
    /**
     * Get current driving choice
     */
    int GetDrivingChoice() const;
    
    // ============================================
    // Wheel Control
    // ============================================
    // Wheel positions found at 0x385F78-0x385FA8
    
    /**
     * Get wheel position
     * @param wheelIndex 0=FL, 1=FR, 2=RL, 3=RR
     */
    void GetWheelPosition(int wheelIndex, float& x, float& y, float& z) const;
    
    /**
     * Get wheel slip (lateral)
     */
    float GetWheelSlipLateral() const;
    
    /**
     * Get wheel slip (forward)
     */
    float GetWheelSlipForward() const;
    
    // ============================================
    // Data Members (Inferred)
    // ============================================
    
private:
    // Steering data
    float m_steeringAngle;        // Current steering angle
    float m_steeringSensitivity;  // Steering sensitivity (0.000250)
    float m_maxSteeringAngle;     // Maximum steering angle (2.237000)
    
    // Speed data
    float m_currentSpeed;         // Current speed
    float m_targetSpeed;          // Target speed from DriveTarget
    float m_maxSpeed;              // Maximum speed (250.0)
    
    // Throttle/Brake
    float m_throttle;             // Current throttle (0.0-1.0)
    float m_brake;                // Current brake (0.0-1.0)
    
    // Physics constants
    float m_accelerationRate;     // Acceleration rate
    float m_decelerationRate;     // Deceleration rate
    float m_frictionCoefficient;  // Friction (0.330000)
    
    // References
    DriveTarget* m_driveTarget;    // Target to drive towards
    DrivingChoice* m_drivingChoice; // Current driving choice
    PlayerSteering* m_playerSteering; // Player steering reference
    
    // Driver type
    void* m_driver;               // Current driver (RealDriver, DriftDriver, etc.)
    
    // Wheel data
    float m_wheelPositions[4][3]; // Wheel positions [FL,FR,RL,RR][x,y,z]
    float m_wheelSlipLateral;     // Lateral wheel slip
    float m_wheelSlipForward;     // Forward wheel slip
    
    // State
    int m_controllerType;         // Controller type ID
    bool m_isActive;              // Is controller active
    bool m_isOutOfControl;        // Out of control state
};

// Related classes (inferred from strings)

class DriveTarget {
    // Target for driving (position, speed, etc.)
    float m_targetX, m_targetY, m_targetZ;
    float m_targetSpeed;
};

class DrivingChoice {
    // Driving mode/choice selection
    int m_choice;
    float m_parameters[4];
};

