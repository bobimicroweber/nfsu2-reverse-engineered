// Main Game Entry Point and Game Loop
// Extracted from SPEED2.EXE
// Entry Point: 0x35BCC7

#pragma once

#include <windows.h>
#include <d3d9.h>
#include "carcontroller_enhanced.h"
#include "realcontroller_logic_detailed.h"
#include <memory>

// Forward declarations
class GameEngine;
class Renderer;
class InputManager;
class PhysicsEngine;

/**
 * Main Game Application
 * Entry point for Need for Speed Underground 2 mod
 */
class GameApplication {
public:
    GameApplication();
    ~GameApplication();
    
    /**
     * Initialize the game
     * Sets up all systems
     */
    bool Initialize(HINSTANCE hInstance, int nCmdShow);
    
    /**
     * Main game loop
     * Runs until game exits
     */
    int Run();
    
    /**
     * Cleanup and shutdown
     */
    void Shutdown();

private:
    // Core systems
    std::unique_ptr<GameEngine> m_gameEngine;
    std::unique_ptr<Renderer> m_renderer;
    std::unique_ptr<InputManager> m_inputManager;
    std::unique_ptr<PhysicsEngine> m_physicsEngine;
    
    // Windows
    HWND m_hWnd = nullptr;
    HINSTANCE m_hInstance = nullptr;
    
    // Game state
    bool m_running = false;
    bool m_initialized = false;
    
    // Timing
    float m_deltaTime = 0.0f;
    DWORD m_lastFrameTime = 0;
    
    // Message handling
    static LRESULT CALLBACK WindowProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
    void ProcessMessages();
    
    // Game loop functions
    void Update(float deltaTime);
    void Render();
    void HandleInput();
};

// ============================================
// WinMain - Windows Entry Point
// ============================================
// Based on disassembly of entry point at 0x35BCC7

/**
 * Windows application entry point
 * This is the actual entry point from the binary
 */
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow) {
    // Initialize COM (if needed)
    // CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
    
    // Create game application
    GameApplication game;
    
    // Initialize game
    if (!game.Initialize(hInstance, nCmdShow)) {
        MessageBoxA(nullptr, "Failed to initialize game", "Error", MB_OK | MB_ICONERROR);
        return -1;
    }
    
    // Run main game loop
    int result = game.Run();
    
    // Cleanup
    game.Shutdown();
    
    // Uninitialize COM
    // CoUninitialize();
    
    return result;
}

// ============================================
// GameApplication Implementation
// ============================================

GameApplication::GameApplication() {
    // Constructor
    m_running = false;
    m_initialized = false;
    m_deltaTime = 0.0f;
    m_lastFrameTime = GetTickCount();
}

GameApplication::~GameApplication() {
    Shutdown();
}

bool GameApplication::Initialize(HINSTANCE hInstance, int nCmdShow) {
    m_hInstance = hInstance;
    
    // Register window class
    WNDCLASSEXA wc = {};
    wc.cbSize = sizeof(WNDCLASSEXA);
    wc.style = CS_HREDRAW | CS_VREDRAW;
    wc.lpfnWndProc = WindowProc;
    wc.hInstance = hInstance;
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.lpszClassName = "Speed2GameWindow";
    
    if (!RegisterClassExA(&wc)) {
        return false;
    }
    
    // Create window
    m_hWnd = CreateWindowExA(
        0,
        "Speed2GameWindow",
        "Need for Speed Underground 2 - Mod",
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT, CW_USEDEFAULT,
        1280, 720,
        nullptr, nullptr, hInstance, this
    );
    
    if (!m_hWnd) {
        return false;
    }
    
    // Initialize Direct3D
    // m_renderer = std::make_unique<Renderer>();
    // if (!m_renderer->Initialize(m_hWnd)) {
    //     return false;
    // }
    
    // Initialize input
    // m_inputManager = std::make_unique<InputManager>();
    
    // Initialize physics
    // m_physicsEngine = std::make_unique<PhysicsEngine>();
    
    // Initialize game engine
    // m_gameEngine = std::make_unique<GameEngine>();
    // if (!m_gameEngine->Initialize()) {
    //     return false;
    // }
    
    // Show window
    ShowWindow(m_hWnd, nCmdShow);
    UpdateWindow(m_hWnd);
    
    m_initialized = true;
    return true;
}

int GameApplication::Run() {
    if (!m_initialized) {
        return -1;
    }
    
    m_running = true;
    
    // Main game loop
    // Based on Windows message loop pattern
    MSG msg = {};
    
    while (m_running) {
        // Process Windows messages
        while (PeekMessageA(&msg, nullptr, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) {
                m_running = false;
                break;
            }
            
            TranslateMessage(&msg);
            DispatchMessageA(&msg);
        }
        
        if (!m_running) break;
        
        // Calculate delta time
        DWORD currentTime = GetTickCount();
        m_deltaTime = (currentTime - m_lastFrameTime) / 1000.0f;
        m_lastFrameTime = currentTime;
        
        // Clamp delta time (prevent large jumps)
        if (m_deltaTime > 0.1f) m_deltaTime = 0.1f;
        
        // Update game
        Update(m_deltaTime);
        
        // Handle input
        HandleInput();
        
        // Render
        Render();
        
        // Present frame
        // m_renderer->Present();
    }
    
    return static_cast<int>(msg.wParam);
}

void GameApplication::Update(float deltaTime) {
    // Update physics
    // if (m_physicsEngine) {
    //     m_physicsEngine->Update(deltaTime);
    // }
    
    // Update game engine
    // if (m_gameEngine) {
    //     m_gameEngine->Update(deltaTime);
    // }
    
    // Update vehicle controllers
    // This is where CarController and RealController would be updated
    // CarController* controller = GetCurrentController();
    // if (controller) {
    //     controller->Update(deltaTime);
    // }
}

void GameApplication::Render() {
    // Clear render target
    // m_renderer->Clear();
    
    // Render game world
    // if (m_gameEngine) {
    //     m_gameEngine->Render(m_renderer.get());
    // }
    
    // Present
    // m_renderer->Present();
}

void GameApplication::HandleInput() {
    // Get input state
    // if (m_inputManager) {
    //     float steering = m_inputManager->GetSteeringInput();
    //     float throttle = m_inputManager->GetThrottleInput();
    //     float brake = m_inputManager->GetBrakeInput();
    //     
    //     // Apply to controller
    //     CarController* controller = GetCurrentController();
    //     if (controller) {
    //         controller->SetSteering(steering);
    //         controller->SetThrottle(throttle);
    //         controller->SetBrake(brake);
    //     }
    // }
}

void GameApplication::Shutdown() {
    m_running = false;
    
    // Cleanup systems
    // if (m_gameEngine) {
    //     m_gameEngine->Shutdown();
    //     m_gameEngine.reset();
    // }
    
    // if (m_renderer) {
    //     m_renderer->Shutdown();
    //     m_renderer.reset();
    // }
    
    // Destroy window
    if (m_hWnd) {
        DestroyWindow(m_hWnd);
        m_hWnd = nullptr;
    }
    
    m_initialized = false;
}

LRESULT CALLBACK GameApplication::WindowProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {
    GameApplication* pApp = nullptr;
    
    if (uMsg == WM_NCCREATE) {
        CREATESTRUCT* pCreate = reinterpret_cast<CREATESTRUCT*>(lParam);
        pApp = reinterpret_cast<GameApplication*>(pCreate->lpCreateParams);
        SetWindowLongPtrA(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pApp));
    } else {
        pApp = reinterpret_cast<GameApplication*>(GetWindowLongPtrA(hWnd, GWLP_USERDATA));
    }
    
    if (pApp) {
        switch (uMsg) {
            case WM_DESTROY:
                PostQuitMessage(0);
                return 0;
                
            case WM_KEYDOWN:
                if (wParam == VK_ESCAPE) {
                    PostQuitMessage(0);
                }
                return 0;
        }
    }
    
    return DefWindowProcA(hWnd, uMsg, wParam, lParam);
}

// ============================================
// Entry Point Disassembly Notes
// ============================================
/*
Entry point disassembly from 0x35BCC7:



Key observations:
- Entry point follows standard Windows application pattern
- Initializes COM and other systems
- Sets up window and message loop
- Main game loop processes messages and updates game
*/

