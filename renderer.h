// Renderer Class
// Extracted from SPEED2.EXE
// Direct3D 9 rendering engine for Need for Speed Underground 2

#pragma once

#include <windows.h>
#include <d3d9.h>
#include <d3dx9.h>
#include <memory>
#include <vector>

// Forward declarations
class Camera;
class Mesh;
class Texture;
class Shader;
class Model;

/**
 * Renderer - Direct3D 9 rendering engine
 * 
 * Handles:
 * - Direct3D device management
 * - Scene rendering
 * - Texture and shader management
 * - Camera and viewport setup
 * - Primitive rendering
 */
class Renderer {
public:
    Renderer();
    ~Renderer();
    
    // ============================================
    // Initialization and Shutdown
    // ============================================
    
    /**
     * Initialize renderer
     * Creates Direct3D device
     */
    bool Initialize(HWND hWnd, int width, int height, bool fullscreen = false);
    
    /**
     * Shutdown renderer
     * Releases all resources
     */
    void Shutdown();
    
    // ============================================
    // Frame Rendering
    // ============================================
    
    /**
     * Begin frame rendering
     * Clears render target and begins scene
     */
    bool BeginFrame();
    
    /**
     * End frame rendering
     * Ends scene and presents
     */
    void EndFrame();
    
    /**
     * Clear render target
     */
    void Clear(float r = 0.0f, float g = 0.0f, float b = 0.0f, float a = 1.0f);
    
    /**
     * Present frame
     */
    void Present();
    
    // ============================================
    // Device Management
    // ============================================
    
    /**
     * Get Direct3D device
     */
    IDirect3DDevice9* GetDevice() const { return m_device; }
    
    /**
     * Get Direct3D interface
     */
    IDirect3D9* GetD3D() const { return m_d3d; }
    
    /**
     * Reset device (for window resize, etc.)
     */
    bool ResetDevice(int width, int height, bool fullscreen = false);
    
    // ============================================
    // Viewport and Camera
    // ============================================
    
    /**
     * Set viewport
     */
    void SetViewport(int x, int y, int width, int height);
    
    /**
     * Get viewport
     */
    void GetViewport(D3DVIEWPORT9& viewport) const;
    
    /**
     * Set camera
     */
    void SetCamera(Camera* camera);
    
    /**
     * Get current camera
     */
    Camera* GetCamera() const { return m_camera; }
    
    // ============================================
    // Transform Matrices
    // ============================================
    
    /**
     * Set world matrix
     */
    void SetWorldMatrix(const D3DXMATRIX& matrix);
    
    /**
     * Set view matrix
     */
    void SetViewMatrix(const D3DXMATRIX& matrix);
    
    /**
     * Set projection matrix
     */
    void SetProjectionMatrix(const D3DXMATRIX& matrix);
    
    /**
     * Get world matrix
     */
    void GetWorldMatrix(D3DXMATRIX& matrix) const;
    
    /**
     * Get view matrix
     */
    void GetViewMatrix(D3DXMATRIX& matrix) const;
    
    /**
     * Get projection matrix
     */
    void GetProjectionMatrix(D3DXMATRIX& matrix) const;
    
    // ============================================
    // Primitive Rendering
    // ============================================
    
    /**
     * Draw primitive
     */
    void DrawPrimitive(D3DPRIMITIVETYPE type, UINT startVertex, UINT primitiveCount);
    
    /**
     * Draw indexed primitive
     */
    void DrawIndexedPrimitive(D3DPRIMITIVETYPE type, UINT minIndex, UINT numVertices,
                              UINT startIndex, UINT primitiveCount);
    
    /**
     * Draw mesh
     */
    void DrawMesh(Mesh* mesh);
    
    /**
     * Draw model
     */
    void DrawModel(Model* model);
    
    // ============================================
    // State Management
    // ============================================
    
    /**
     * Set render state
     */
    void SetRenderState(D3DRENDERSTATETYPE state, DWORD value);
    
    /**
     * Get render state
     */
    DWORD GetRenderState(D3DRENDERSTATETYPE state) const;
    
    /**
     * Set texture
     */
    void SetTexture(DWORD stage, IDirect3DTexture9* texture);
    
    /**
     * Set texture stage state
     */
    void SetTextureStageState(DWORD stage, D3DTEXTURESTAGESTATETYPE type, DWORD value);
    
    // ============================================
    // Resource Management
    // ============================================
    
    /**
     * Create texture
     */
    IDirect3DTexture9* CreateTexture(UINT width, UINT height, D3DFORMAT format);
    
    /**
     * Load texture from file
     */
    IDirect3DTexture9* LoadTexture(const char* filename);
    
    /**
     * Create vertex buffer
     */
    IDirect3DVertexBuffer9* CreateVertexBuffer(UINT size, DWORD usage, DWORD fvf);
    
    /**
     * Create index buffer
     */
    IDirect3DIndexBuffer9* CreateIndexBuffer(UINT size, DWORD usage, D3DFORMAT format);
    
    // ============================================
    // Properties
    // ============================================
    
    int GetWidth() const { return m_width; }
    int GetHeight() const { return m_height; }
    bool IsFullscreen() const { return m_fullscreen; }
    bool IsInitialized() const { return m_initialized; }

private:
    // Direct3D objects
    IDirect3D9* m_d3d = nullptr;
    IDirect3DDevice9* m_device = nullptr;
    
    // Window
    HWND m_hWnd = nullptr;
    int m_width = 0;
    int m_height = 0;
    bool m_fullscreen = false;
    
    // Camera
    Camera* m_camera = nullptr;
    
    // Matrices
    D3DXMATRIX m_worldMatrix;
    D3DXMATRIX m_viewMatrix;
    D3DXMATRIX m_projectionMatrix;
    
    // Viewport
    D3DVIEWPORT9 m_viewport;
    
    // State
    bool m_initialized = false;
    bool m_inScene = false;
    
    // Presentation parameters
    D3DPRESENT_PARAMETERS m_presentParams;
    
    // Internal methods
    bool CreateDevice();
    void ReleaseDevice();
    void SetupDefaultStates();
    void SetupMatrices();
};

// ============================================
// Camera Class
// ============================================

/**
 * Camera - Represents a 3D camera
 */
class Camera {
public:
    Camera();
    ~Camera();
    
    // Position and target
    void SetPosition(float x, float y, float z);
    void GetPosition(float& x, float& y, float& z) const {
        x = m_x; y = m_y; z = m_z;
    }
    
    void SetTarget(float x, float y, float z);
    void GetTarget(float& x, float& y, float& z) const {
        x = m_targetX; y = m_targetY; z = m_targetZ;
    }
    
    void SetUp(float x, float y, float z);
    void GetUp(float& x, float& y, float& z) const {
        x = m_upX; y = m_upY; z = m_upZ;
    }
    
    // Projection
    void SetFOV(float fov) { m_fov = fov; UpdateProjection(); }
    float GetFOV() const { return m_fov; }
    
    void SetNearPlane(float nearPlane) { m_nearPlane = nearPlane; UpdateProjection(); }
    float GetNearPlane() const { return m_nearPlane; }
    
    void SetFarPlane(float farPlane) { m_farPlane = farPlane; UpdateProjection(); }
    float GetFarPlane() const { return m_farPlane; }
    
    void SetAspectRatio(float aspect) { m_aspectRatio = aspect; UpdateProjection(); }
    float GetAspectRatio() const { return m_aspectRatio; }
    
    // Matrices
    void GetViewMatrix(D3DXMATRIX& matrix) const;
    void GetProjectionMatrix(D3DXMATRIX& matrix) const;
    
    // Update
    void Update();

private:
    // Position
    float m_x = 0.0f, m_y = 0.0f, m_z = -10.0f;
    float m_targetX = 0.0f, m_targetY = 0.0f, m_targetZ = 0.0f;
    float m_upX = 0.0f, m_upY = 1.0f, m_upZ = 0.0f;
    
    // Projection
    float m_fov = D3DX_PI / 4.0f;  // 45 degrees
    float m_nearPlane = 0.1f;
    float m_farPlane = 1000.0f;
    float m_aspectRatio = 16.0f / 9.0f;
    
    // Matrices
    D3DXMATRIX m_viewMatrix;
    D3DXMATRIX m_projectionMatrix;
    
    void UpdateView();
    void UpdateProjection();
};

// ============================================
// Renderer Implementation
// ============================================

inline Renderer::Renderer() {
    ZeroMemory(&m_presentParams, sizeof(m_presentParams));
    ZeroMemory(&m_viewport, sizeof(m_viewport));
    D3DXMatrixIdentity(&m_worldMatrix);
    D3DXMatrixIdentity(&m_viewMatrix);
    D3DXMatrixIdentity(&m_projectionMatrix);
}

inline Renderer::~Renderer() {
    Shutdown();
}

inline bool Renderer::Initialize(HWND hWnd, int width, int height, bool fullscreen) {
    if (m_initialized) {
        return true;
    }
    
    m_hWnd = hWnd;
    m_width = width;
    m_height = height;
    m_fullscreen = fullscreen;
    
    // Create Direct3D
    m_d3d = Direct3DCreate9(D3D_SDK_VERSION);
    if (!m_d3d) {
        return false;
    }
    
    // Setup presentation parameters
    ZeroMemory(&m_presentParams, sizeof(m_presentParams));
    m_presentParams.Windowed = !fullscreen;
    m_presentParams.SwapEffect = D3DSWAPEFFECT_DISCARD;
    m_presentParams.hDeviceWindow = hWnd;
    m_presentParams.BackBufferFormat = fullscreen ? D3DFMT_X8R8G8B8 : D3DFMT_UNKNOWN;
    m_presentParams.BackBufferWidth = width;
    m_presentParams.BackBufferHeight = height;
    m_presentParams.EnableAutoDepthStencil = TRUE;
    m_presentParams.AutoDepthStencilFormat = D3DFMT_D24S8;
    m_presentParams.PresentationInterval = D3DPRESENT_INTERVAL_ONE;  // VSync
    
    // Create device
    if (!CreateDevice()) {
        return false;
    }
    
    // Setup default states
    SetupDefaultStates();
    
    // Setup matrices
    SetupMatrices();
    
    // Setup viewport
    SetViewport(0, 0, width, height);
    
    m_initialized = true;
    return true;
}

inline bool Renderer::CreateDevice() {
    D3DDEVTYPE deviceType = D3DDEVTYPE_HAL;
    DWORD behaviorFlags = D3DCREATE_HARDWARE_VERTEXPROCESSING;
    
    // Try hardware vertex processing first
    HRESULT hr = m_d3d->CreateDevice(
        D3DADAPTER_DEFAULT,
        deviceType,
        m_hWnd,
        behaviorFlags,
        &m_presentParams,
        &m_device
    );
    
    // Fallback to software if hardware fails
    if (FAILED(hr)) {
        behaviorFlags = D3DCREATE_SOFTWARE_VERTEXPROCESSING;
        hr = m_d3d->CreateDevice(
            D3DADAPTER_DEFAULT,
            deviceType,
            m_hWnd,
            behaviorFlags,
            &m_presentParams,
            &m_device
        );
    }
    
    return SUCCEEDED(hr);
}

inline void Renderer::SetupDefaultStates() {
    if (!m_device) return;
    
    // Lighting
    m_device->SetRenderState(D3DRS_LIGHTING, FALSE);
    
    // Culling
    m_device->SetRenderState(D3DRS_CULLMODE, D3DCULL_CCW);
    
    // Z-Buffer
    m_device->SetRenderState(D3DRS_ZENABLE, TRUE);
    m_device->SetRenderState(D3DRS_ZFUNC, D3DCMP_LESSEQUAL);
    
    // Alpha blending
    m_device->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE);
    
    // Fog
    m_device->SetRenderState(D3DRS_FOGENABLE, FALSE);
}

inline void Renderer::SetupMatrices() {
    if (!m_device) return;
    
    // Set matrices
    m_device->SetTransform(D3DTS_WORLD, &m_worldMatrix);
    m_device->SetTransform(D3DTS_VIEW, &m_viewMatrix);
    m_device->SetTransform(D3DTS_PROJECTION, &m_projectionMatrix);
}

inline bool Renderer::BeginFrame() {
    if (!m_device || m_inScene) {
        return false;
    }
    
    HRESULT hr = m_device->BeginScene();
    if (FAILED(hr)) {
        return false;
    }
    
    m_inScene = true;
    return true;
}

inline void Renderer::EndFrame() {
    if (!m_device || !m_inScene) {
        return;
    }
    
    m_device->EndScene();
    m_inScene = false;
}

inline void Renderer::Clear(float r, float g, float b, float a) {
    if (!m_device) return;
    
    DWORD color = D3DCOLOR_COLORVALUE(r, g, b, a);
    m_device->Clear(0, nullptr, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, color, 1.0f, 0);
}

inline void Renderer::Present() {
    if (!m_device) return;
    
    HRESULT hr = m_device->Present(nullptr, nullptr, nullptr, nullptr);
    
    // Handle device lost
    if (hr == D3DERR_DEVICELOST) {
        // Device lost, need to reset
    }
}

inline void Renderer::SetViewport(int x, int y, int width, int height) {
    if (!m_device) return;
    
    m_viewport.X = x;
    m_viewport.Y = y;
    m_viewport.Width = width;
    m_viewport.Height = height;
    m_viewport.MinZ = 0.0f;
    m_viewport.MaxZ = 1.0f;
    
    m_device->SetViewport(&m_viewport);
}

inline void Renderer::GetViewport(D3DVIEWPORT9& viewport) const {
    viewport = m_viewport;
}

inline void Renderer::SetCamera(Camera* camera) {
    m_camera = camera;
    if (m_camera) {
        m_camera->Update();
        m_camera->GetViewMatrix(m_viewMatrix);
        m_camera->GetProjectionMatrix(m_projectionMatrix);
        SetupMatrices();
    }
}

inline void Renderer::SetWorldMatrix(const D3DXMATRIX& matrix) {
    m_worldMatrix = matrix;
    if (m_device) {
        m_device->SetTransform(D3DTS_WORLD, &m_worldMatrix);
    }
}

inline void Renderer::SetViewMatrix(const D3DXMATRIX& matrix) {
    m_viewMatrix = matrix;
    if (m_device) {
        m_device->SetTransform(D3DTS_VIEW, &m_viewMatrix);
    }
}

inline void Renderer::SetProjectionMatrix(const D3DXMATRIX& matrix) {
    m_projectionMatrix = matrix;
    if (m_device) {
        m_device->SetTransform(D3DTS_PROJECTION, &m_projectionMatrix);
    }
}

inline void Renderer::GetWorldMatrix(D3DXMATRIX& matrix) const {
    matrix = m_worldMatrix;
}

inline void Renderer::GetViewMatrix(D3DXMATRIX& matrix) const {
    matrix = m_viewMatrix;
}

inline void Renderer::GetProjectionMatrix(D3DXMATRIX& matrix) const {
    matrix = m_projectionMatrix;
}

inline void Renderer::DrawPrimitive(D3DPRIMITIVETYPE type, UINT startVertex, UINT primitiveCount) {
    if (!m_device || !m_inScene) return;
    m_device->DrawPrimitive(type, startVertex, primitiveCount);
}

inline void Renderer::DrawIndexedPrimitive(D3DPRIMITIVETYPE type, UINT minIndex, UINT numVertices,
                                          UINT startIndex, UINT primitiveCount) {
    if (!m_device || !m_inScene) return;
    m_device->DrawIndexedPrimitive(type, minIndex, numVertices, startIndex, primitiveCount);
}

inline void Renderer::SetRenderState(D3DRENDERSTATETYPE state, DWORD value) {
    if (!m_device) return;
    m_device->SetRenderState(state, value);
}

inline DWORD Renderer::GetRenderState(D3DRENDERSTATETYPE state) const {
    if (!m_device) return 0;
    DWORD value = 0;
    m_device->GetRenderState(state, &value);
    return value;
}

inline void Renderer::SetTexture(DWORD stage, IDirect3DTexture9* texture) {
    if (!m_device) return;
    m_device->SetTexture(stage, texture);
}

inline void Renderer::SetTextureStageState(DWORD stage, D3DTEXTURESTAGESTATETYPE type, DWORD value) {
    if (!m_device) return;
    m_device->SetTextureStageState(stage, type, value);
}

inline bool Renderer::ResetDevice(int width, int height, bool fullscreen) {
    if (!m_device) return false;
    
    m_width = width;
    m_height = height;
    m_fullscreen = fullscreen;
    
    m_presentParams.BackBufferWidth = width;
    m_presentParams.BackBufferHeight = height;
    m_presentParams.Windowed = !fullscreen;
    
    HRESULT hr = m_device->Reset(&m_presentParams);
    if (SUCCEEDED(hr)) {
        SetupDefaultStates();
        SetupMatrices();
        SetViewport(0, 0, width, height);
        return true;
    }
    
    return false;
}

inline void Renderer::Shutdown() {
    if (m_inScene) {
        EndFrame();
    }
    
    ReleaseDevice();
    
    if (m_d3d) {
        m_d3d->Release();
        m_d3d = nullptr;
    }
    
    m_initialized = false;
}

inline void Renderer::ReleaseDevice() {
    if (m_device) {
        m_device->Release();
        m_device = nullptr;
    }
}

// Camera implementation
inline Camera::Camera() {
    UpdateView();
    UpdateProjection();
}

inline Camera::~Camera() {
}

inline void Camera::SetPosition(float x, float y, float z) {
    m_x = x; m_y = y; m_z = z;
    UpdateView();
}

inline void Camera::SetTarget(float x, float y, float z) {
    m_targetX = x; m_targetY = y; m_targetZ = z;
    UpdateView();
}

inline void Camera::SetUp(float x, float y, float z) {
    m_upX = x; m_upY = y; m_upZ = z;
    UpdateView();
}

inline void Camera::UpdateView() {
    D3DXVECTOR3 eye(m_x, m_y, m_z);
    D3DXVECTOR3 target(m_targetX, m_targetY, m_targetZ);
    D3DXVECTOR3 up(m_upX, m_upY, m_upZ);
    D3DXMatrixLookAtLH(&m_viewMatrix, &eye, &target, &up);
}

inline void Camera::UpdateProjection() {
    D3DXMatrixPerspectiveFovLH(&m_projectionMatrix, m_fov, m_aspectRatio, m_nearPlane, m_farPlane);
}

inline void Camera::GetViewMatrix(D3DXMATRIX& matrix) const {
    matrix = m_viewMatrix;
}

inline void Camera::GetProjectionMatrix(D3DXMATRIX& matrix) const {
    matrix = m_projectionMatrix;
}

inline void Camera::Update() {
    UpdateView();
    UpdateProjection();
}

