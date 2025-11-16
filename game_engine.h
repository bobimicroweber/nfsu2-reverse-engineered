// GameEngine Class
// Extracted from SPEED2.EXE
// Core game engine for Need for Speed Underground 2 mod

#pragma once

#include <windows.h>
#include <memory>
#include <vector>
#include <string>
#include "carcontroller_enhanced.h"
#include "realcontroller_logic_detailed.h"

// Forward declarations
class Renderer;
class InputManager;
class PhysicsEngine;
class AudioEngine;
class World;
class Scene;
class Vehicle;
class Camera;

/**
 * GameEngine - Core game engine class
 * 
 * Manages all game systems and coordinates between them.
 * Handles initialization, update loop, and rendering coordination.
 */
class GameEngine {
public:
    GameEngine();
    ~GameEngine();
    
    // ============================================
    // Initialization and Shutdown
    // ============================================
    
    /**
     * Initialize the game engine
     * Sets up all subsystems
     */
    bool Initialize();
    
    /**
     * Shutdown the game engine
     * Cleans up all resources
     */
    void Shutdown();
    
    // ============================================
    // Main Game Loop
    // ============================================
    
    /**
     * Update game engine
     * Called every frame
     */
    void Update(float deltaTime);
    
    /**
     * Render game engine
     * Coordinates rendering of all systems
     */
    void Render(Renderer* renderer);
    
    // ============================================
    // World and Scene Management
    // ============================================
    
    /**
     * Load a world/level
     */
    bool LoadWorld(const std::string& worldName);
    
    /**
     * Unload current world
     */
    void UnloadWorld();
    
    /**
     * Get current world
     */
    World* GetCurrentWorld() const { return m_currentWorld.get(); }
    
    /**
     * Create a new scene
     */
    Scene* CreateScene(const std::string& sceneName);
    
    /**
     * Get current scene
     */
    Scene* GetCurrentScene() const { return m_currentScene.get(); }
    
    // ============================================
    // Vehicle Management
    // ============================================
    
    /**
     * Create a vehicle
     */
    Vehicle* CreateVehicle(const std::string& vehicleName);
    
    /**
     * Get player vehicle
     */
    Vehicle* GetPlayerVehicle() const { return m_playerVehicle.get(); }
    
    /**
     * Set player vehicle
     */
    void SetPlayerVehicle(Vehicle* vehicle);
    
    // ============================================
    // System Access
    // ============================================
    
    PhysicsEngine* GetPhysicsEngine() const { return m_physicsEngine.get(); }
    AudioEngine* GetAudioEngine() const { return m_audioEngine.get(); }
    InputManager* GetInputManager() const { return m_inputManager.get(); }
    
    // ============================================
    // Game State
    // ============================================
    
    enum class GameState {
        Uninitialized,
        Initializing,
        MainMenu,
        Loading,
        InGame,
        Paused,
        ShuttingDown
    };
    
    GameState GetState() const { return m_gameState; }
    void SetState(GameState state) { m_gameState = state; }
    
    bool IsRunning() const { return m_running; }
    void SetRunning(bool running) { m_running = running; }

private:
    // Core systems
    std::unique_ptr<PhysicsEngine> m_physicsEngine;
    std::unique_ptr<AudioEngine> m_audioEngine;
    std::unique_ptr<InputManager> m_inputManager;
    
    // World and scene
    std::unique_ptr<World> m_currentWorld;
    std::unique_ptr<Scene> m_currentScene;
    
    // Vehicles
    std::unique_ptr<Vehicle> m_playerVehicle;
    std::vector<std::unique_ptr<Vehicle>> m_vehicles;
    
    // Game state
    GameState m_gameState = GameState::Uninitialized;
    bool m_running = false;
    bool m_initialized = false;
    
    // Timing
    float m_totalTime = 0.0f;
    float m_frameTime = 0.0f;
    
    // Initialization helpers
    bool InitializeSystems();
    bool InitializeWorld();
    void ShutdownSystems();
};

// ============================================
// World Class
// ============================================

/**
 * World - Represents a game world/level
 */
class World {
public:
    World(const std::string& name);
    ~World();
    
    bool Load();
    void Unload();
    
    void Update(float deltaTime);
    void Render(Renderer* renderer);
    
    const std::string& GetName() const { return m_name; }
    bool IsLoaded() const { return m_loaded; }

private:
    std::string m_name;
    bool m_loaded = false;
    
    // World data
    // - Terrain
    // - Objects
    // - Lighting
    // - Physics boundaries
};

// ============================================
// Scene Class
// ============================================

/**
 * Scene - Represents a game scene
 */
class Scene {
public:
    Scene(const std::string& name);
    ~Scene();
    
    void Update(float deltaTime);
    void Render(Renderer* renderer);
    
    const std::string& GetName() const { return m_name; }

private:
    std::string m_name;
    
    // Scene data
    // - Objects
    // - Camera
    // - Lighting
};

// ============================================
// Vehicle Class
// ============================================

/**
 * Vehicle - Represents a vehicle in the game
 */
class Vehicle {
public:
    Vehicle(const std::string& name);
    ~Vehicle();
    
    void Update(float deltaTime);
    void Render(Renderer* renderer);
    
    // Controller access
    CarController* GetController() { return m_controller.get(); }
    RealController* GetRealController() { return m_realController.get(); }
    
    void SetControllerType(bool useRealistic) {
        m_useRealisticController = useRealistic;
    }
    
    const std::string& GetName() const { return m_name; }

private:
    std::string m_name;
    
    // Controllers
    std::unique_ptr<CarController> m_controller;
    std::unique_ptr<RealController> m_realController;
    bool m_useRealisticController = false;
    
    // Vehicle data
    // - Position
    // - Rotation
    // - Model
    // - Physics properties
};

// ============================================
// GameEngine Implementation
// ============================================

inline GameEngine::GameEngine() {
    m_gameState = GameState::Uninitialized;
    m_running = false;
    m_initialized = false;
    m_totalTime = 0.0f;
    m_frameTime = 0.0f;
}

inline GameEngine::~GameEngine() {
    Shutdown();
}

inline bool GameEngine::Initialize() {
    if (m_initialized) {
        return true;
    }
    
    m_gameState = GameState::Initializing;
    
    // Initialize core systems
    if (!InitializeSystems()) {
        return false;
    }
    
    // Initialize world
    if (!InitializeWorld()) {
        return false;
    }
    
    m_initialized = true;
    m_gameState = GameState::MainMenu;
    m_running = true;
    
    return true;
}

inline bool GameEngine::InitializeSystems() {
    // Initialize physics engine
    m_physicsEngine = std::make_unique<PhysicsEngine>();
    if (!m_physicsEngine->Initialize()) {
        return false;
    }
    
    // Initialize audio engine
    m_audioEngine = std::make_unique<AudioEngine>();
    if (!m_audioEngine->Initialize()) {
        return false;
    }
    
    // Initialize input manager
    m_inputManager = std::make_unique<InputManager>();
    if (!m_inputManager->Initialize()) {
        return false;
    }
    
    return true;
}

inline bool GameEngine::InitializeWorld() {
    // Create default world or load from file
    // For now, create empty world
    m_currentWorld = std::make_unique<World>("DefaultWorld");
    
    return true;
}

inline void GameEngine::Update(float deltaTime) {
    if (!m_initialized || !m_running) {
        return;
    }
    
    m_frameTime = deltaTime;
    m_totalTime += deltaTime;
    
    // Update input
    if (m_inputManager) {
        m_inputManager->Update();
    }
    
    // Update physics
    if (m_physicsEngine) {
        m_physicsEngine->Update(deltaTime);
    }
    
    // Update world
    if (m_currentWorld) {
        m_currentWorld->Update(deltaTime);
    }
    
    // Update scene
    if (m_currentScene) {
        m_currentScene->Update(deltaTime);
    }
    
    // Update vehicles
    if (m_playerVehicle) {
        // Get input
        float steering = 0.0f;
        float throttle = 0.0f;
        float brake = 0.0f;
        
        if (m_inputManager) {
            steering = m_inputManager->GetSteeringInput();
            throttle = m_inputManager->GetThrottleInput();
            brake = m_inputManager->GetBrakeInput();
        }
        
        // Apply to controller
        if (m_playerVehicle->GetRealController()) {
            RealController* controller = m_playerVehicle->GetRealController();
            controller->ApplyRealisticSteering(steering, deltaTime);
            controller->ApplyRealisticAcceleration(throttle, deltaTime);
            controller->ApplyRealisticBraking(brake, deltaTime);
            controller->UpdateRealisticPhysics(deltaTime);
        } else if (m_playerVehicle->GetController()) {
            CarController* controller = m_playerVehicle->GetController();
            controller->SetSteering(steering);
            controller->SetThrottle(throttle);
            controller->SetBrake(brake);
            controller->Update(deltaTime);
        }
        
        m_playerVehicle->Update(deltaTime);
    }
    
    // Update other vehicles
    for (auto& vehicle : m_vehicles) {
        if (vehicle) {
            vehicle->Update(deltaTime);
        }
    }
    
    // Update audio
    if (m_audioEngine) {
        m_audioEngine->Update(deltaTime);
    }
}

inline void GameEngine::Render(Renderer* renderer) {
    if (!m_initialized || !renderer) {
        return;
    }
    
    // Render world
    if (m_currentWorld) {
        m_currentWorld->Render(renderer);
    }
    
    // Render scene
    if (m_currentScene) {
        m_currentScene->Render(renderer);
    }
    
    // Render vehicles
    if (m_playerVehicle) {
        m_playerVehicle->Render(renderer);
    }
    
    for (auto& vehicle : m_vehicles) {
        if (vehicle) {
            vehicle->Render(renderer);
        }
    }
}

inline bool GameEngine::LoadWorld(const std::string& worldName) {
    // Unload current world
    UnloadWorld();
    
    // Create new world
    m_currentWorld = std::make_unique<World>(worldName);
    
    // Load world data
    if (!m_currentWorld->Load()) {
        m_currentWorld.reset();
        return false;
    }
    
    return true;
}

inline void GameEngine::UnloadWorld() {
    if (m_currentWorld) {
        m_currentWorld->Unload();
        m_currentWorld.reset();
    }
}

inline Scene* GameEngine::CreateScene(const std::string& sceneName) {
    m_currentScene = std::make_unique<Scene>(sceneName);
    return m_currentScene.get();
}

inline Vehicle* GameEngine::CreateVehicle(const std::string& vehicleName) {
    auto vehicle = std::make_unique<Vehicle>(vehicleName);
    Vehicle* ptr = vehicle.get();
    m_vehicles.push_back(std::move(vehicle));
    return ptr;
}

inline void GameEngine::SetPlayerVehicle(Vehicle* vehicle) {
    // Find vehicle in list
    for (auto& v : m_vehicles) {
        if (v.get() == vehicle) {
            m_playerVehicle = std::move(v);
            break;
        }
    }
}

inline void GameEngine::Shutdown() {
    m_running = false;
    m_gameState = GameState::ShuttingDown;
    
    // Unload world
    UnloadWorld();
    
    // Clear vehicles
    m_playerVehicle.reset();
    m_vehicles.clear();
    
    // Shutdown systems
    ShutdownSystems();
    
    m_initialized = false;
    m_gameState = GameState::Uninitialized;
}

inline void GameEngine::ShutdownSystems() {
    if (m_audioEngine) {
        m_audioEngine->Shutdown();
        m_audioEngine.reset();
    }
    
    if (m_inputManager) {
        m_inputManager->Shutdown();
        m_inputManager.reset();
    }
    
    if (m_physicsEngine) {
        m_physicsEngine->Shutdown();
        m_physicsEngine.reset();
    }
}

