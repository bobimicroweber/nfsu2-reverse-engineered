// InputManager Class
// Extracted from SPEED2.EXE
// Input handling for keyboard, mouse, gamepad, and steering wheel

#pragma once

#include <windows.h>
#include <dinput.h>
#include <XInput.h>
#include <unordered_map>
#include <string>

// Forward declarations
class Vehicle;

/**
 * InputManager - Handles all input devices
 * 
 * Supports:
 * - Keyboard input
 * - Mouse input
 * - Gamepad/Controller input (XInput)
 * - Steering wheel input
 * - DirectInput devices
 */
class InputManager {
public:
    InputManager();
    ~InputManager();
    
    // ============================================
    // Initialization and Shutdown
    // ============================================
    
    /**
     * Initialize input manager
     */
    bool Initialize(HWND hWnd);
    
    /**
     * Shutdown input manager
     */
    void Shutdown();
    
    // ============================================
    // Update
    // ============================================
    
    /**
     * Update input state
     * Called every frame
     */
    void Update();
    
    // ============================================
    // Keyboard Input
    // ============================================
    
    /**
     * Check if key is currently pressed
     */
    bool IsKeyPressed(int virtualKey) const;
    
    /**
     * Check if key was just pressed (this frame)
     */
    bool IsKeyJustPressed(int virtualKey) const;
    
    /**
     * Check if key was just released (this frame)
     */
    bool IsKeyJustReleased(int virtualKey) const;
    
    // Common key checks
    bool IsSteeringLeft() const;
    bool IsSteeringRight() const;
    bool IsThrottle() const;
    bool IsBrake() const;
    bool IsHandbrake() const;
    bool IsNitro() const;
    bool IsShiftUp() const;
    bool IsShiftDown() const;
    
    // ============================================
    // Mouse Input
    // ============================================
    
    /**
     * Get mouse position
     */
    void GetMousePosition(int& x, int& y) const;
    
    /**
     * Get mouse delta (movement since last frame)
     */
    void GetMouseDelta(int& dx, int& dy) const;
    
    /**
     * Check if mouse button is pressed
     */
    bool IsMouseButtonPressed(int button) const;
    
    // ============================================
    // Gamepad/Controller Input (XInput)
    // ============================================
    
    /**
     * Check if gamepad is connected
     */
    bool IsGamepadConnected(int playerIndex = 0) const;
    
    /**
     * Get left stick X axis (-1.0 to 1.0)
     */
    float GetLeftStickX(int playerIndex = 0) const;
    
    /**
     * Get left stick Y axis (-1.0 to 1.0)
     */
    float GetLeftStickY(int playerIndex = 0) const;
    
    /**
     * Get right stick X axis (-1.0 to 1.0)
     */
    float GetRightStickX(int playerIndex = 0) const;
    
    /**
     * Get right stick Y axis (-1.0 to 1.0)
     */
    float GetRightStickY(int playerIndex = 0) const;
    
    /**
     * Get left trigger (0.0 to 1.0)
     */
    float GetLeftTrigger(int playerIndex = 0) const;
    
    /**
     * Get right trigger (0.0 to 1.0)
     */
    float GetRightTrigger(int playerIndex = 0) const;
    
    /**
     * Check if gamepad button is pressed
     */
    bool IsGamepadButtonPressed(int button, int playerIndex = 0) const;
    
    // ============================================
    // Vehicle Input (Normalized)
    // ============================================
    
    /**
     * Get steering input (-1.0 to 1.0)
     * Combines keyboard, gamepad, and wheel input
     */
    float GetSteeringInput() const;
    
    /**
     * Get throttle input (0.0 to 1.0)
     * Combines keyboard, gamepad, and wheel input
     */
    float GetThrottleInput() const;
    
    /**
     * Get brake input (0.0 to 1.0)
     * Combines keyboard, gamepad, and wheel input
     */
    float GetBrakeInput() const;
    
    /**
     * Get handbrake input (0.0 to 1.0)
     */
    float GetHandbrakeInput() const;
    
    /**
     * Get nitro input (0.0 to 1.0)
     */
    float GetNitroInput() const;
    
    // ============================================
    // Input Mapping
    // ============================================
    
    /**
     * Set key mapping
     */
    void SetKeyMapping(const std::string& action, int virtualKey);
    
    /**
     * Get key mapping
     */
    int GetKeyMapping(const std::string& action) const;
    
    // ============================================
    // Properties
    // ============================================
    
    bool IsInitialized() const { return m_initialized; }
    int GetActiveInputDevice() const { return m_activeDevice; }
    
    enum InputDevice {
        DEVICE_KEYBOARD = 0,
        DEVICE_GAMEPAD = 1,
        DEVICE_WHEEL = 2,
        DEVICE_MOUSE = 3
    };
    
    void SetActiveDevice(InputDevice device) { m_activeDevice = device; }

private:
    // Window
    HWND m_hWnd = nullptr;
    
    // DirectInput
    LPDIRECTINPUT8 m_directInput = nullptr;
    LPDIRECTINPUTDEVICE8 m_keyboard = nullptr;
    LPDIRECTINPUTDEVICE8 m_mouse = nullptr;
    LPDIRECTINPUTDEVICE8 m_gamepad = nullptr;
    
    // Keyboard state
    BYTE m_keyboardState[256] = {0};
    BYTE m_previousKeyboardState[256] = {0};
    
    // Mouse state
    DIMOUSESTATE m_mouseState = {};
    DIMOUSESTATE m_previousMouseState = {};
    POINT m_mousePosition = {0, 0};
    POINT m_previousMousePosition = {0, 0};
    
    // XInput gamepad state
    XINPUT_STATE m_gamepadState[4] = {};
    XINPUT_STATE m_previousGamepadState[4] = {};
    bool m_gamepadConnected[4] = {false, false, false, false};
    
    // Input mappings
    std::unordered_map<std::string, int> m_keyMappings;
    
    // Active device
    InputDevice m_activeDevice = DEVICE_KEYBOARD;
    
    // State
    bool m_initialized = false;
    
    // Internal methods
    bool InitializeDirectInput();
    void UpdateKeyboard();
    void UpdateMouse();
    void UpdateGamepad();
    void DetectActiveDevice();
    
    // Key mappings (from binary)
    void SetupDefaultKeyMappings();
};

// ============================================
// InputManager Implementation
// ============================================

inline InputManager::InputManager() {
    m_initialized = false;
    m_activeDevice = DEVICE_KEYBOARD;
    ZeroMemory(m_keyboardState, sizeof(m_keyboardState));
    ZeroMemory(m_previousKeyboardState, sizeof(m_previousKeyboardState));
    ZeroMemory(&m_mouseState, sizeof(m_mouseState));
    ZeroMemory(&m_previousMouseState, sizeof(m_previousMouseState));
    SetupDefaultKeyMappings();
}

inline InputManager::~InputManager() {
    Shutdown();
}

inline bool InputManager::Initialize(HWND hWnd) {
    if (m_initialized) {
        return true;
    }
    
    m_hWnd = hWnd;
    
    // Initialize DirectInput
    if (!InitializeDirectInput()) {
        return false;
    }
    
    // Setup default key mappings
    SetupDefaultKeyMappings();
    
    m_initialized = true;
    return true;
}

inline bool InputManager::InitializeDirectInput() {
    // Create DirectInput interface
    HRESULT hr = DirectInput8Create(
        GetModuleHandle(nullptr),
        DIRECTINPUT_VERSION,
        IID_IDirectInput8,
        (void**)&m_directInput,
        nullptr
    );
    
    if (FAILED(hr)) {
        return false;
    }
    
    // Create keyboard device
    hr = m_directInput->CreateDevice(GUID_SysKeyboard, &m_keyboard, nullptr);
    if (SUCCEEDED(hr)) {
        m_keyboard->SetDataFormat(&c_dfDIKeyboard);
        m_keyboard->SetCooperativeLevel(m_hWnd, DISCL_FOREGROUND | DISCL_NONEXCLUSIVE);
        m_keyboard->Acquire();
    }
    
    // Create mouse device
    hr = m_directInput->CreateDevice(GUID_SysMouse, &m_mouse, nullptr);
    if (SUCCEEDED(hr)) {
        m_mouse->SetDataFormat(&c_dfDIMouse);
        m_mouse->SetCooperativeLevel(m_hWnd, DISCL_FOREGROUND | DISCL_NONEXCLUSIVE);
        m_mouse->Acquire();
    }
    
    return true;
}

inline void InputManager::SetupDefaultKeyMappings() {
    // Default key mappings (based on common game controls)
    m_keyMappings["SteerLeft"] = 'A';
    m_keyMappings["SteerRight"] = 'D';
    m_keyMappings["Throttle"] = 'W';
    m_keyMappings["Brake"] = 'S';
    m_keyMappings["Handbrake"] = VK_SPACE;
    m_keyMappings["Nitro"] = VK_SHIFT;
    m_keyMappings["ShiftUp"] = VK_UP;
    m_keyMappings["ShiftDown"] = VK_DOWN;
}

inline void InputManager::Update() {
    if (!m_initialized) return;
    
    // Save previous states
    memcpy(m_previousKeyboardState, m_keyboardState, sizeof(m_keyboardState));
    m_previousMouseState = m_mouseState;
    m_previousMousePosition = m_mousePosition;
    for (int i = 0; i < 4; ++i) {
        m_previousGamepadState[i] = m_gamepadState[i];
    }
    
    // Update input devices
    UpdateKeyboard();
    UpdateMouse();
    UpdateGamepad();
    
    // Detect active input device
    DetectActiveDevice();
}

inline void InputManager::UpdateKeyboard() {
    if (!m_keyboard) return;
    
    HRESULT hr = m_keyboard->GetDeviceState(sizeof(m_keyboardState), m_keyboardState);
    if (FAILED(hr)) {
        // Try to reacquire
        m_keyboard->Acquire();
        m_keyboard->GetDeviceState(sizeof(m_keyboardState), m_keyboardState);
    }
}

inline void InputManager::UpdateMouse() {
    if (!m_mouse) return;
    
    HRESULT hr = m_mouse->GetDeviceState(sizeof(DIMOUSESTATE), &m_mouseState);
    if (FAILED(hr)) {
        // Try to reacquire
        m_mouse->Acquire();
        m_mouse->GetDeviceState(sizeof(DIMOUSESTATE), &m_mouseState);
    }
    
    // Update mouse position
    GetCursorPos(&m_mousePosition);
    ScreenToClient(m_hWnd, &m_mousePosition);
}

inline void InputManager::UpdateGamepad() {
    // Update XInput gamepads
    for (int i = 0; i < 4; ++i) {
        XINPUT_STATE state;
        ZeroMemory(&state, sizeof(XINPUT_STATE));
        
        DWORD result = XInputGetState(i, &state);
        if (result == ERROR_SUCCESS) {
            m_gamepadState[i] = state;
            m_gamepadConnected[i] = true;
        } else {
            m_gamepadConnected[i] = false;
        }
    }
}

inline void InputManager::DetectActiveDevice() {
    // Detect which device is being used
    // Check gamepad first
    if (IsGamepadConnected(0)) {
        // Check if any gamepad input
        if (std::abs(GetLeftStickX(0)) > 0.1f ||
            std::abs(GetLeftStickY(0)) > 0.1f ||
            GetLeftTrigger(0) > 0.1f ||
            GetRightTrigger(0) > 0.1f) {
            m_activeDevice = DEVICE_GAMEPAD;
            return;
        }
    }
    
    // Check keyboard
    if (IsKeyPressed('W') || IsKeyPressed('S') || 
        IsKeyPressed('A') || IsKeyPressed('D')) {
        m_activeDevice = DEVICE_KEYBOARD;
        return;
    }
    
    // Default to keyboard
    m_activeDevice = DEVICE_KEYBOARD;
}

inline bool InputManager::IsKeyPressed(int virtualKey) const {
    return (m_keyboardState[virtualKey] & 0x80) != 0;
}

inline bool InputManager::IsKeyJustPressed(int virtualKey) const {
    return IsKeyPressed(virtualKey) && 
           (m_previousKeyboardState[virtualKey] & 0x80) == 0;
}

inline bool InputManager::IsKeyJustReleased(int virtualKey) const {
    return !IsKeyPressed(virtualKey) && 
           (m_previousKeyboardState[virtualKey] & 0x80) != 0;
}

inline bool InputManager::IsSteeringLeft() const {
    return IsKeyPressed(m_keyMappings.at("SteerLeft"));
}

inline bool InputManager::IsSteeringRight() const {
    return IsKeyPressed(m_keyMappings.at("SteerRight"));
}

inline bool InputManager::IsThrottle() const {
    return IsKeyPressed(m_keyMappings.at("Throttle"));
}

inline bool InputManager::IsBrake() const {
    return IsKeyPressed(m_keyMappings.at("Brake"));
}

inline bool InputManager::IsHandbrake() const {
    return IsKeyPressed(m_keyMappings.at("Handbrake"));
}

inline bool InputManager::IsNitro() const {
    return IsKeyPressed(m_keyMappings.at("Nitro"));
}

inline bool InputManager::IsShiftUp() const {
    return IsKeyPressed(m_keyMappings.at("ShiftUp"));
}

inline bool InputManager::IsShiftDown() const {
    return IsKeyPressed(m_keyMappings.at("ShiftDown"));
}

inline void InputManager::GetMousePosition(int& x, int& y) const {
    x = m_mousePosition.x;
    y = m_mousePosition.y;
}

inline void InputManager::GetMouseDelta(int& dx, int& dy) const {
    dx = m_mouseState.lX;
    dy = m_mouseState.lY;
}

inline bool InputManager::IsMouseButtonPressed(int button) const {
    return (m_mouseState.rgbButtons[button] & 0x80) != 0;
}

inline bool InputManager::IsGamepadConnected(int playerIndex) const {
    if (playerIndex < 0 || playerIndex >= 4) return false;
    return m_gamepadConnected[playerIndex];
}

inline float InputManager::GetLeftStickX(int playerIndex) const {
    if (!IsGamepadConnected(playerIndex)) return 0.0f;
    
    SHORT x = m_gamepadState[playerIndex].Gamepad.sThumbLX;
    float normalized = x / 32768.0f;
    
    // Apply dead zone
    if (std::abs(normalized) < 0.1f) return 0.0f;
    
    return normalized;
}

inline float InputManager::GetLeftStickY(int playerIndex) const {
    if (!IsGamepadConnected(playerIndex)) return 0.0f;
    
    SHORT y = m_gamepadState[playerIndex].Gamepad.sThumbLY;
    float normalized = y / 32768.0f;
    
    // Apply dead zone
    if (std::abs(normalized) < 0.1f) return 0.0f;
    
    return normalized;
}

inline float InputManager::GetRightStickX(int playerIndex) const {
    if (!IsGamepadConnected(playerIndex)) return 0.0f;
    
    SHORT x = m_gamepadState[playerIndex].Gamepad.sThumbRX;
    float normalized = x / 32768.0f;
    
    // Apply dead zone
    if (std::abs(normalized) < 0.1f) return 0.0f;
    
    return normalized;
}

inline float InputManager::GetRightStickY(int playerIndex) const {
    if (!IsGamepadConnected(playerIndex)) return 0.0f;
    
    SHORT y = m_gamepadState[playerIndex].Gamepad.sThumbRY;
    float normalized = y / 32768.0f;
    
    // Apply dead zone
    if (std::abs(normalized) < 0.1f) return 0.0f;
    
    return normalized;
}

inline float InputManager::GetLeftTrigger(int playerIndex) const {
    if (!IsGamepadConnected(playerIndex)) return 0.0f;
    
    BYTE trigger = m_gamepadState[playerIndex].Gamepad.bLeftTrigger;
    return trigger / 255.0f;
}

inline float InputManager::GetRightTrigger(int playerIndex) const {
    if (!IsGamepadConnected(playerIndex)) return 0.0f;
    
    BYTE trigger = m_gamepadState[playerIndex].Gamepad.bRightTrigger;
    return trigger / 255.0f;
}

inline bool InputManager::IsGamepadButtonPressed(int button, int playerIndex) const {
    if (!IsGamepadConnected(playerIndex)) return false;
    
    return (m_gamepadState[playerIndex].Gamepad.wButtons & button) != 0;
}

inline float InputManager::GetSteeringInput() const {
    float steering = 0.0f;
    
    // Check gamepad first (if connected and active)
    if (m_activeDevice == DEVICE_GAMEPAD && IsGamepadConnected(0)) {
        steering = GetLeftStickX(0);
    }
    // Check keyboard
    else if (m_activeDevice == DEVICE_KEYBOARD) {
        if (IsSteeringLeft()) steering = -1.0f;
        if (IsSteeringRight()) steering = 1.0f;
    }
    
    return steering;
}

inline float InputManager::GetThrottleInput() const {
    float throttle = 0.0f;
    
    // Check gamepad
    if (m_activeDevice == DEVICE_GAMEPAD && IsGamepadConnected(0)) {
        throttle = GetRightTrigger(0);
    }
    // Check keyboard
    else if (m_activeDevice == DEVICE_KEYBOARD) {
        if (IsThrottle()) throttle = 1.0f;
    }
    
    return throttle;
}

inline float InputManager::GetBrakeInput() const {
    float brake = 0.0f;
    
    // Check gamepad
    if (m_activeDevice == DEVICE_GAMEPAD && IsGamepadConnected(0)) {
        brake = GetLeftTrigger(0);
    }
    // Check keyboard
    else if (m_activeDevice == DEVICE_KEYBOARD) {
        if (IsBrake()) brake = 1.0f;
    }
    
    return brake;
}

inline float InputManager::GetHandbrakeInput() const {
    float handbrake = 0.0f;
    
    // Check gamepad
    if (m_activeDevice == DEVICE_GAMEPAD && IsGamepadConnected(0)) {
        // Handbrake typically mapped to a button (e.g., X button)
        if (IsGamepadButtonPressed(XINPUT_GAMEPAD_X, 0)) {
            handbrake = 1.0f;
        }
    }
    // Check keyboard
    else if (m_activeDevice == DEVICE_KEYBOARD) {
        if (IsHandbrake()) handbrake = 1.0f;
    }
    
    return handbrake;
}

inline float InputManager::GetNitroInput() const {
    float nitro = 0.0f;
    
    // Check gamepad
    if (m_activeDevice == DEVICE_GAMEPAD && IsGamepadConnected(0)) {
        // Nitro typically mapped to a button (e.g., A button)
        if (IsGamepadButtonPressed(XINPUT_GAMEPAD_A, 0)) {
            nitro = 1.0f;
        }
    }
    // Check keyboard
    else if (m_activeDevice == DEVICE_KEYBOARD) {
        if (IsNitro()) nitro = 1.0f;
    }
    
    return nitro;
}

inline void InputManager::SetKeyMapping(const std::string& action, int virtualKey) {
    m_keyMappings[action] = virtualKey;
}

inline int InputManager::GetKeyMapping(const std::string& action) const {
    auto it = m_keyMappings.find(action);
    return (it != m_keyMappings.end()) ? it->second : 0;
}

inline void InputManager::Shutdown() {
    // Release DirectInput devices
    if (m_keyboard) {
        m_keyboard->Unacquire();
        m_keyboard->Release();
        m_keyboard = nullptr;
    }
    
    if (m_mouse) {
        m_mouse->Unacquire();
        m_mouse->Release();
        m_mouse = nullptr;
    }
    
    if (m_gamepad) {
        m_gamepad->Unacquire();
        m_gamepad->Release();
        m_gamepad = nullptr;
    }
    
    if (m_directInput) {
        m_directInput->Release();
        m_directInput = nullptr;
    }
    
    m_initialized = false;
}

