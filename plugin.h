#pragma once
#include "bakkesmod/plugin/bakkesmodplugin.h"
#include "bakkesmod/plugin/pluginwindow.h"
#include "bakkesmod/plugin/PluginSettingsWindow.h"
#include <chrono>

struct TargetOrientation {
    Rotator targetRotator;
    Vector targetForward;
    Vector targetUp;
    Vector ballPosition;
    bool hasTarget = false;
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
    void OnPhysicsTick(std::string eventName);
    void GenerateNewTarget(CarWrapper car);
    bool CheckAlignment(CarWrapper car, float deltaTime);

    // Settings
    bool enabled = false;
    bool requireRollAlignment = false;
    float noseToleranceDegrees = 18.0f;
    float rollToleranceDegrees = 25.0f;
    float holdDuration = 0.10f;
    float anchorZ = 600.0f;
    float targetDistance = 800.0f;

    // Telemetry
    float currentNoseErrorDeg = 0.0f;
    float currentRollErrorDeg = 0.0f;

    // Runtime
    bool initializedTarget = false;
    Vector anchorPosition = { 0.f, 0.f, 600.f };
    TargetOrientation currentTarget;
    float currentHoldTime = 0.0f;
    std::chrono::steady_clock::time_point targetStartTime;
    float lastReactionTimeMs = 0.0f;
    int targetsCompleted = 0;
};