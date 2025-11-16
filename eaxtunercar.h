#ifndef EAXTUNERCAR_H
#define EAXTUNERCAR_H


#include <windows.h>
#include <dsound.h>

// Forward declarations
class Vehicle;
class Car;

/**
 * EAXTunerCar
 * Audio class for handling eaxtunercar audio
 * Location in binary: 0x389C84
 */
class EAXTunerCar : public EAXAudioObject
{
public:
    EAXTunerCar();
    virtual ~EAXTunerCar();
    virtual float GetPosition() const;
    virtual float GetVolume() const;
    virtual void Initialize();
    virtual bool IsPaused() const;
    virtual bool IsPlaying() const;
    virtual void Load();
    virtual void Pause();
    virtual void Play();
    virtual void Resume();
    virtual void SetPosition(float value);
    virtual void SetVolume(float value);
    virtual void Shutdown();
    virtual void Stop();
    virtual void Unload();
    virtual void Update();

    // EAX-specific methods
    virtual void SetEAXProperties(const EAX_PROPERTIES& props);
    virtual void GetEAXProperties(EAX_PROPERTIES& props) const;
    virtual void SetReverb(float reverb);
    virtual void SetEcho(float echo);
    virtual void SetEnvironment(int environment);

    // Vehicle-specific methods
    virtual void AttachToVehicle(Vehicle* vehicle);
    virtual void DetachFromVehicle();
    virtual Vehicle* GetVehicle() const;
    virtual void UpdateEngineSound(float rpm, float throttle);
    virtual void UpdateTireSound(float speed, float slip);
    virtual void UpdateCollisionSound(float impact);

protected:
    // Audio properties
    float m_volume;
    float m_pitch;
    bool m_isPlaying;
    bool m_isPaused;
    D3DVECTOR m_position;
    D3DVECTOR m_velocity;
    
    // EAX properties
    EAX_PROPERTIES m_eaxProperties;
    float m_reverb;
    float m_echo;
    int m_environment;
    // Vehicle reference
    Vehicle* m_vehicle;
    float m_engineRPM;
    float m_throttle;
    float m_tireSpeed;
    float m_tireSlip;

    // DirectSound
    LPDIRECTSOUNDBUFFER m_soundBuffer;
    LPDIRECTSOUND3DBUFFER8 m_3dBuffer;
};

// EAX Properties structure
struct EAX_PROPERTIES
{
    float fEnvironment;
    float fEnvironmentSize;
    float fEnvironmentDiffusion;
    int lRoom;
    int lRoomHF;
    int lRoomLF;
    int flDecayTime;
    float flDecayHFRatio;
    float flDecayLFRatio;
    int lReflections;
    float flReflectionsDelay;
    int lReverb;
    float flReverbDelay;
    int lRoomRolloffFactor;
    int lAirAbsorptionHF;
    int lFlags;
};

#endif // EAXTUNERCAR_H
