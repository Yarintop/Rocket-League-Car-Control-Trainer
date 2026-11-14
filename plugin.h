#pragma once
#include "bakkesmod/plugin/bakkesmodplugin.h"
#include "bakkesmod/plugin/pluginwindow.h"
#include "bakkesmod/plugin/PluginSettingsWindow.h"
#include <chrono>
#include <array>
#include <string>

enum class TrainerMode : int {
    InvertedRelativeSim = 0, // Car centered; relative physics moves the ball
    FreeFlightGates = 1      // Fly through gates around the pitch
};

enum class RollPhaseMode : int {
    AnyRoll = 0,
    CardinalPhases = 1 // Upright (0°), Right (90°), Inverted (180°), Left (270°)
};

struct AerialTarget {
    Vector position = { 0.f, 0.f, 0.f };
    Vector targetUp = { 0.f, 0.f, 1.f };
    int phaseIndex = 0; // 0: Upright, 1: Right, 2: Inverted, 3: Left
    bool active = false;
};

class Plugin : public BakkesMod::Plugin::BakkesModPlugin, public BakkesMod::Plugin::PluginSettingsWindow
{
public:
    virtual void onLoad() override;
    virtual void onUnload() override;

    void Render(CanvasWrapper canvas);
    void RenderSettings() override;
    std::string GetPluginName() override;
    void SetImGuiContext(uintptr_t ctx) override;

private:
    void OnPhysicsTick();
    void SpawnNextTarget(CarWrapper car);
    void ResetRelativeSim(CarWrapper car);
    void CheckTargetHit(CarWrapper car);
    void RenderAnimeSpeedLines(CanvasWrapper canvas, const Vector& carPos, const Vector& streamDir, float currentSpeed);

    // Settings
    bool enabled = false;
    int currentMode = static_cast<int>(TrainerMode::InvertedRelativeSim);
    int rollPhaseMode = static_cast<int>(RollPhaseMode::CardinalPhases);
    
    float targetHitRadius = 180.0f;
    float rollToleranceDegrees = 35.0f;
    float relativeSpawnDistance = 1400.0f;
    float gameSpeed = 1.0f;
    bool showStickGuide = true;

    // Anime Speed Lines Settings
    bool showAnimeSpeedLines = true;
    int maxAnimeLines = 45;
    float animeLineLengthBase = 320.0f;
    float animeLineThickness = 2.5f;

    // Inverted Relative Sim state
    Vector simAnchorPos = { 0.f, 0.f, 1000.f };
    Vector relBallPos = { 0.f, 1200.f, 1000.f };
    Vector prevBallPos = { 0.f, 1200.f, 1000.f };
    Vector relBallVel = { 0.f, 0.f, 0.f };

    // Telemetry & Metrics
    AerialTarget target;
    float currentRollErrorDeg = 0.0f;
    float localPitchDeflection = 0.0f;
    float localYawDeflection = 0.0f;
    int targetsHit = 0;
    float lastHitTimeMs = 0.0f;
    std::chrono::steady_clock::time_point targetStartTime;

    std::array<int, 4> phaseHitCounts = { 0, 0, 0, 0 };
    std::array<float, 4> phaseTotalTimeMs = { 0.0f, 0.0f, 0.0f, 0.0f };
};