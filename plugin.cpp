#include "pch.h"
#include "Plugin.h"
#include <cmath>
#include <algorithm>

BAKKESMOD_PLUGIN(Plugin, "Car Control Orientation Trainer", "2.6", PLUGINTYPE_FREEPLAY)

std::shared_ptr<CVarManagerWrapper> _globalCvarManager;

constexpr float PI_F = 3.14159265358979323846f;
constexpr float TWO_PI_F = 2.0f * PI_F;
constexpr float RAD_TO_DEG = 180.0f / PI_F;
constexpr float DEG_TO_RAD = PI_F / 180.0f;
constexpr float UROT_TO_RAD = PI_F / 32768.0f;

// Rocket League physics constants
constexpr float RL_GRAVITY_Z = 650.0f;
constexpr float RL_BOOST_ACCEL = 1060.0f;

// Arena bounds
constexpr float CEILING_Z = 1950.0f;
constexpr float FLOOR_Z   = 95.0f;
constexpr float SIDE_WALL_X = 4000.0f;
constexpr float BACK_WALL_Y = 5020.0f;
constexpr float GOAL_X_LIMIT = 900.0f;
constexpr float GOAL_Z_LIMIT = 645.0f;

static float VectorDot(const Vector& a, const Vector& b) {
    return (a.X * b.X) + (a.Y * b.Y) + (a.Z * b.Z);
}

static float VectorMagnitude(const Vector& v) {
    return std::sqrt((v.X * v.X) + (v.Y * v.Y) + (v.Z * v.Z));
}

static Vector VectorNormalize(const Vector& v) {
    float mag = VectorMagnitude(v);
    if (mag > 0.00001f) {
        return Vector{ v.X / mag, v.Y / mag, v.Z / mag };
    }
    return Vector{ 0.f, 0.f, 0.f };
}

static Vector VectorCross(const Vector& a, const Vector& b) {
    return Vector{
        a.Y * b.Z - a.Z * b.Y,
        a.Z * b.X - a.X * b.Z,
        a.X * b.Y - a.Y * b.X
    };
}

static void GetRotatorAxes(const Rotator& r, Vector& outForward, Vector& outRight, Vector& outUp) {
    float pitchRad = static_cast<float>(r.Pitch) * UROT_TO_RAD;
    float yawRad   = static_cast<float>(r.Yaw)   * UROT_TO_RAD;
    float rollRad  = static_cast<float>(r.Roll)  * UROT_TO_RAD;

    float cp = std::cos(pitchRad);
    float sp = std::sin(pitchRad);
    float cy = std::cos(yawRad);
    float sy = std::sin(yawRad);
    float cr = std::cos(rollRad);
    float sr = std::sin(rollRad);

    outForward = Vector{ cp * cy, cp * sy, sp };
    outRight   = Vector{ cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, -cp * sr };
    outUp      = Vector{ -cy * sp * cr - sy * sr, -sy * sp * cr + cy * sr, cp * cr };
}

static float DistPointToSegment(const Vector& point, const Vector& segA, const Vector& segB) {
    Vector ab = Vector{ segB.X - segA.X, segB.Y - segA.Y, segB.Z - segA.Z };
    Vector ap = Vector{ point.X - segA.X, point.Y - segA.Y, point.Z - segA.Z };
    float abLenSq = (ab.X * ab.X) + (ab.Y * ab.Y) + (ab.Z * ab.Z);
    if (abLenSq < 0.0001f) return VectorMagnitude(ap);

    float t = std::clamp(((ap.X * ab.X) + (ap.Y * ab.Y) + (ap.Z * ab.Z)) / abLenSq, 0.0f, 1.0f);
    Vector closest = Vector{ segA.X + (ab.X * t), segA.Y + (ab.Y * t), segA.Z + (ab.Z * t) };
    return VectorMagnitude(Vector{ point.X - closest.X, point.Y - closest.Y, point.Z - closest.Z });
}

static void DrawThickLineIn2D(CanvasWrapper& canvas, Vector2 p1, Vector2 p2, float thickness) {
    float dx = static_cast<float>(p2.X - p1.X);
    float dy = static_cast<float>(p2.Y - p1.Y);
    float len = std::sqrt(dx * dx + dy * dy);
    if (len < 0.001f) return;

    float normX = -dy / len;
    float normY = dx / len;

    int half = static_cast<int>(thickness / 2.0f);
    for (int i = -half; i <= half; ++i) {
        int offX = static_cast<int>(std::round(normX * static_cast<float>(i)));
        int offY = static_cast<int>(std::round(normY * static_cast<float>(i)));
        canvas.DrawLine(Vector2{ p1.X + offX, p1.Y + offY }, Vector2{ p2.X + offX, p2.Y + offY });
    }
}

std::string Plugin::GetPluginName() {
    return "Car Control Trainer";
}

void Plugin::SetImGuiContext(uintptr_t ctx) {
    ImGui::SetCurrentContext(reinterpret_cast<ImGuiContext*>(ctx));
}

void Plugin::onLoad() {
    _globalCvarManager = cvarManager;

    gameWrapper->HookEvent(
        "Function TAGame.Car_TA.SetVehicleInput",
        [this](std::string eventName) { OnPhysicsTick(); }
    );

    gameWrapper->RegisterDrawable([this](CanvasWrapper canvas) { Render(canvas); });
}

void Plugin::onUnload() {
    gameWrapper->UnhookEvent("Function TAGame.Car_TA.SetVehicleInput");
    if (_globalCvarManager) {
        auto speedCvar = _globalCvarManager->getCvar("sv_soccar_gamespeed");
        if (speedCvar) speedCvar.setValue(1.0f);
    }
}

void Plugin::ResetRelativeSim(CarWrapper car) {
    simAnchorPos = Vector{ 0.f, 0.f, 1000.f };
    car.SetLocation(simAnchorPos);
    car.SetVelocity(Vector{ 0.f, 0.f, 0.f });

    float theta = (rand() % 360) * DEG_TO_RAD;
    float phi = ((rand() % 120) - 60.0f) * DEG_TO_RAD;

    Vector spawnDir = VectorNormalize(Vector{
        std::cos(theta) * std::cos(phi),
        std::sin(theta) * std::cos(phi),
        std::sin(phi)
    });

    relBallPos = Vector{
        std::clamp(simAnchorPos.X + (spawnDir.X * relativeSpawnDistance), -SIDE_WALL_X + 200.0f, SIDE_WALL_X - 200.0f),
        std::clamp(simAnchorPos.Y + (spawnDir.Y * relativeSpawnDistance), -BACK_WALL_Y + 250.0f, BACK_WALL_Y - 250.0f),
        std::clamp(simAnchorPos.Z + (spawnDir.Z * relativeSpawnDistance), FLOOR_Z + 150.0f, CEILING_Z - 150.0f)
    };
    prevBallPos = relBallPos;
    relBallVel = Vector{ 0.f, 0.f, 0.f };

    target.position = relBallPos;
    target.active = true;
    targetStartTime = std::chrono::steady_clock::now();
}

void Plugin::SpawnNextTarget(CarWrapper car) {
    targetStartTime = std::chrono::steady_clock::now();

    target.phaseIndex = (rollPhaseMode == static_cast<int>(RollPhaseMode::CardinalPhases)) ? (rand() % 4) : 0;

    Vector worldUp = { 0.f, 0.f, 1.f };
    Vector worldRight = { 1.f, 0.f, 0.f };

    switch (target.phaseIndex) {
        case 0: target.targetUp = worldUp; break;
        case 1: target.targetUp = worldRight; break;
        case 2: target.targetUp = Vector{ 0.f, 0.f, -1.f }; break;
        case 3: target.targetUp = Vector{ -1.f, 0.f, 0.f }; break;
    }

    if (currentMode == static_cast<int>(TrainerMode::InvertedRelativeSim)) {
        ResetRelativeSim(car);
    } else {
        Vector carPos = car.GetLocation();
        float angle = (rand() % 360) * DEG_TO_RAD;
        float dist = 1200.0f + (rand() % 900);
        float targetZ = 600.0f + (rand() % 800);

        target.position = Vector{
            std::clamp(carPos.X + (std::cos(angle) * dist), -3000.0f, 3000.0f),
            std::clamp(carPos.Y + (std::sin(angle) * dist), -4000.0f, 4000.0f),
            targetZ
        };
        prevBallPos = target.position;
        target.active = true;
    }
}

void Plugin::CheckTargetHit(CarWrapper car) {
    Vector carPos = car.GetLocation();

    float dist = (currentMode == static_cast<int>(TrainerMode::InvertedRelativeSim))
        ? DistPointToSegment(carPos, prevBallPos, relBallPos)
        : VectorMagnitude(target.position - carPos);

    float effectiveRadius = targetHitRadius + 93.0f;

    if (dist <= effectiveRadius) {
        bool rollValid = true;
        if (rollPhaseMode == static_cast<int>(RollPhaseMode::CardinalPhases)) {
            Rotator carRot = car.GetRotation();
            Vector carFwd, carRight, carUp;
            GetRotatorAxes(carRot, carFwd, carRight, carUp);

            Vector normCarUp = VectorNormalize(carUp);
            Vector normTargetUp = VectorNormalize(target.targetUp);
            float dot = std::clamp(VectorDot(normCarUp, normTargetUp), -1.0f, 1.0f);
            currentRollErrorDeg = std::acos(dot) * RAD_TO_DEG;
            rollValid = (currentRollErrorDeg <= rollToleranceDegrees);
        }

        if (rollValid) {
            auto now = std::chrono::steady_clock::now();
            lastHitTimeMs = std::chrono::duration<float, std::milli>(now - targetStartTime).count();
            targetsHit++;

            if (target.phaseIndex >= 0 && target.phaseIndex < 4) {
                phaseHitCounts[target.phaseIndex]++;
                phaseTotalTimeMs[target.phaseIndex] += lastHitTimeMs;
            }

            SpawnNextTarget(car);
        } else {
            // Wrong orientation upon contact: spawn next target to keep momentum fluid
            SpawnNextTarget(car);
        }
    }
}

void Plugin::RenderAnimeSpeedLines(CanvasWrapper canvas, const Vector& carPos, const Vector& streamDir, float currentSpeed) {
    if (!showAnimeSpeedLines || currentSpeed < 250.0f) return;

    Vector2 screenSize = canvas.GetSize();
    if (screenSize.X <= 0 || screenSize.Y <= 0) return;

    Vector2 screenCenter = { static_cast<int>(screenSize.X / 2.0f), static_cast<int>(screenSize.Y / 2.0f) };

    Vector horizonTarget = Vector{
        carPos.X + (streamDir.X * 1800.0f),
        carPos.Y + (streamDir.Y * 1800.0f),
        carPos.Z + (streamDir.Z * 1800.0f)
    };
    Vector2 projVanishingPoint = canvas.Project(horizonTarget);

    Vector2 focusPoint = screenCenter;
    if (projVanishingPoint.X > -screenSize.X && projVanishingPoint.X < screenSize.X * 2 &&
        projVanishingPoint.Y > -screenSize.Y && projVanishingPoint.Y < screenSize.Y * 2) {
        focusPoint = projVanishingPoint;
    }

    float speedRatio = std::clamp((currentSpeed - 250.0f) / 1600.0f, 0.1f, 1.0f);
    int activeLineCount = static_cast<int>(static_cast<float>(maxAnimeLines) * speedRatio);

    float innerClearRadius = 160.0f;
    float diagonal = static_cast<float>(std::sqrt(static_cast<double>(screenSize.X * screenSize.X + screenSize.Y * screenSize.Y)));

    for (int i = 0; i < activeLineCount; ++i) {
        float angle = ((rand() % 3600) / 10.0f) * DEG_TO_RAD;
        float dirX = std::cos(angle);
        float dirY = std::sin(angle);

        int maxRandRange = std::max(1, static_cast<int>(diagonal * 0.5f));
        float outerDist = innerClearRadius + 180.0f + static_cast<float>(rand() % maxRandRange);
        float lineLength = animeLineLengthBase * (0.5f + (static_cast<float>(rand() % 80) / 100.0f)) * speedRatio;
        float innerDist = std::max(innerClearRadius, outerDist - lineLength);

        Vector2 pOuter = {
            static_cast<int>(static_cast<float>(focusPoint.X) + (dirX * outerDist)),
            static_cast<int>(static_cast<float>(focusPoint.Y) + (dirY * outerDist))
        };
        Vector2 pInner = {
            static_cast<int>(static_cast<float>(focusPoint.X) + (dirX * innerDist)),
            static_cast<int>(static_cast<float>(focusPoint.Y) + (dirY * innerDist))
        };

        int alpha = 100 + (rand() % 140);
        if (i % 4 == 0) {
            canvas.SetColor((char)110, (char)225, (char)255, (char)alpha);
        } else {
            canvas.SetColor((char)245, (char)250, (char)255, (char)alpha);
        }

        DrawThickLineIn2D(canvas, pOuter, pInner, animeLineThickness);
    }
}

void Plugin::OnPhysicsTick() {
    if (!enabled || !gameWrapper->IsInFreeplay()) return;

    ServerWrapper server = gameWrapper->GetGameEventAsServer();
    if (server.IsNull()) return;

    CarWrapper car = gameWrapper->GetLocalCar();
    if (car.IsNull()) return;

    if (!target.active) {
        SpawnNextTarget(car);
        return;
    }

    constexpr float dt = 1.0f / 120.0f;
    BallWrapper ball = server.GetBall();

    if (currentMode == static_cast<int>(TrainerMode::InvertedRelativeSim)) {
        car.SetLocation(simAnchorPos);
        car.SetVelocity(Vector{ 0.f, 0.f, 0.f });

        Rotator carRot = car.GetRotation();
        Vector carFwd, carRight, carUp;
        GetRotatorAxes(carRot, carFwd, carRight, carUp);

        // 1. Inverted Gravity: relative ball accelerates upward
        relBallVel.Z += RL_GRAVITY_Z * dt;

        // 2. Inverted Boost
        auto input = car.GetInput();
        if (input.ActivateBoost) {
            relBallVel.X -= carFwd.X * RL_BOOST_ACCEL * dt;
            relBallVel.Y -= carFwd.Y * RL_BOOST_ACCEL * dt;
            relBallVel.Z -= carFwd.Z * RL_BOOST_ACCEL * dt;
        }

        // Aerial drag
        relBallVel.X *= (1.0f - (0.05f * dt));
        relBallVel.Y *= (1.0f - (0.05f * dt));
        relBallVel.Z *= (1.0f - (0.05f * dt));

        prevBallPos = relBallPos;
        relBallPos.X += relBallVel.X * dt;
        relBallPos.Y += relBallVel.Y * dt;
        relBallPos.Z += relBallVel.Z * dt;

        // Ceiling & floor bounce/stop
        if (relBallPos.Z >= CEILING_Z) {
            relBallPos.Z = CEILING_Z;
            if (relBallVel.Z > 0.0f) relBallVel.Z = -relBallVel.Z * 0.2f; // Slight damp bounce
        } else if (relBallPos.Z <= FLOOR_Z) {
            relBallPos.Z = FLOOR_Z;
            if (relBallVel.Z < 0.0f) relBallVel.Z = -relBallVel.Z * 0.2f;
        }

        // Walls damp
        if (relBallPos.X >= SIDE_WALL_X) {
            relBallPos.X = SIDE_WALL_X;
            if (relBallVel.X > 0.0f) relBallVel.X = -relBallVel.X * 0.2f;
        } else if (relBallPos.X <= -SIDE_WALL_X) {
            relBallPos.X = -SIDE_WALL_X;
            if (relBallVel.X < 0.0f) relBallVel.X = -relBallVel.X * 0.2f;
        }

        bool inGoalMouth = (std::abs(relBallPos.X) <= GOAL_X_LIMIT && relBallPos.Z <= GOAL_Z_LIMIT);
        float effectiveBackWallY = inGoalMouth ? (BACK_WALL_Y - 50.0f) : BACK_WALL_Y;

        if (relBallPos.Y >= effectiveBackWallY) {
            relBallPos.Y = effectiveBackWallY;
            if (relBallVel.Y > 0.0f) relBallVel.Y = -relBallVel.Y * 0.2f;
        } else if (relBallPos.Y <= -effectiveBackWallY) {
            relBallPos.Y = -effectiveBackWallY;
            if (relBallVel.Y < 0.0f) relBallVel.Y = -relBallVel.Y * 0.2f;
        }

        target.position = relBallPos;

        if (!ball.IsNull()) {
            ball.SetLocation(relBallPos);
            ball.SetVelocity(Vector{ 0.f, 0.f, 0.f });
            ball.SetAngularVelocity(Vector{ 0.f, 0.f, 0.f }, false);
        }
    } else {
        // Free Flight Mode
        if (!ball.IsNull()) {
            ball.SetLocation(target.position);
            ball.SetVelocity(Vector{ 0.f, 0.f, 0.f });
            ball.SetAngularVelocity(Vector{ 0.f, 0.f, 0.f }, false);
        }
    }

    // Telemetry updates
    Rotator carRot = car.GetRotation();
    Vector carFwd, carRight, carUp;
    GetRotatorAxes(carRot, carFwd, carRight, carUp);

    Vector toTarget = VectorNormalize(target.position - car.GetLocation());
    localPitchDeflection = std::clamp(VectorDot(toTarget, carUp), -1.0f, 1.0f);
    localYawDeflection   = std::clamp(VectorDot(toTarget, carRight), -1.0f, 1.0f);

    Vector normCarUp = VectorNormalize(carUp);
    Vector normTargetUp = VectorNormalize(target.targetUp);
    currentRollErrorDeg = std::acos(std::clamp(VectorDot(normCarUp, normTargetUp), -1.0f, 1.0f)) * RAD_TO_DEG;

    CheckTargetHit(car);
}

void Plugin::Render(CanvasWrapper canvas) {
    if (!enabled || !gameWrapper->IsInFreeplay()) return;

    CarWrapper car = gameWrapper->GetLocalCar();
    if (!car.IsNull() && showAnimeSpeedLines) {
        float effectiveSpeed = 0.0f;
        Vector streamDir = { 0.f, 1.f, 0.f };

        if (currentMode == static_cast<int>(TrainerMode::InvertedRelativeSim)) {
            effectiveSpeed = VectorMagnitude(relBallVel);
            streamDir = (effectiveSpeed > 50.0f) 
                ? VectorNormalize(Vector{ -relBallVel.X, -relBallVel.Y, -relBallVel.Z })
                : VectorNormalize(car.GetVelocity());
        } else {
            Vector carVel = car.GetVelocity();
            effectiveSpeed = VectorMagnitude(carVel);
            if (effectiveSpeed > 50.0f) {
                streamDir = VectorNormalize(carVel);
            } else {
                Vector dummyR, dummyU;
                GetRotatorAxes(car.GetRotation(), streamDir, dummyR, dummyU);
                streamDir = VectorNormalize(streamDir);
            }
        }

        RenderAnimeSpeedLines(canvas, car.GetLocation(), streamDir, effectiveSpeed);
    }

    // Telemetry HUD
    canvas.SetColor((char)255, (char)255, (char)255, (char)255);
    canvas.SetPosition(Vector2{ 40, 50 });
    std::string modeTitle = (currentMode == static_cast<int>(TrainerMode::InvertedRelativeSim))
        ? "DAR Trainer: [INVERTED SPACE RELATIVE]"
        : "DAR Trainer: [FREE FLIGHT GATES]";
    canvas.DrawString(modeTitle, 1.2f, 1.2f);

    canvas.SetPosition(Vector2{ 40, 75 });
    canvas.DrawString("Hits: " + std::to_string(targetsHit) + " | Last: " + 
                      std::to_string(static_cast<int>(lastHitTimeMs)) + " ms", 1.1f, 1.1f);

    if (rollPhaseMode == static_cast<int>(RollPhaseMode::CardinalPhases)) {
        std::string phaseStr = "Target Phase: ";
        if (target.phaseIndex == 0) phaseStr += "UPRIGHT (Hood)";
        else if (target.phaseIndex == 1) phaseStr += "RIGHT FLANK";
        else if (target.phaseIndex == 2) phaseStr += "INVERTED (Belly)";
        else if (target.phaseIndex == 3) phaseStr += "LEFT FLANK";

        if (currentRollErrorDeg <= rollToleranceDegrees) {
            canvas.SetColor((char)0, (char)255, (char)0, (char)255);
        } else {
            canvas.SetColor((char)255, (char)180, (char)180, (char)255);
        }
        canvas.SetPosition(Vector2{ 40, 100 });
        canvas.DrawString(phaseStr + " (" + std::to_string(static_cast<int>(currentRollErrorDeg)) + " deg)", 1.0f, 1.0f);
    }

    if (showStickGuide) {
        Vector2 radarCenter = { 100, 210 };
        float radarRadius = 45.0f;

        canvas.SetColor((char)120, (char)120, (char)120, (char)180);
        constexpr int segments = 16;
        for (int i = 0; i < segments; ++i) {
            float a1 = (TWO_PI_F / segments) * static_cast<float>(i);
            float a2 = (TWO_PI_F / segments) * static_cast<float>(i + 1);
            Vector2 p1 = {
                radarCenter.X + static_cast<int>(std::cos(a1) * radarRadius),
                radarCenter.Y + static_cast<int>(std::sin(a1) * radarRadius)
            };
            Vector2 p2 = {
                radarCenter.X + static_cast<int>(std::cos(a2) * radarRadius),
                radarCenter.Y + static_cast<int>(std::sin(a2) * radarRadius)
            };
            canvas.DrawLine(p1, p2);
        }

        int dotX = radarCenter.X + static_cast<int>(localYawDeflection * radarRadius);
        int dotY = radarCenter.Y - static_cast<int>(localPitchDeflection * radarRadius);
        canvas.SetColor((char)255, (char)220, (char)0, (char)255);
        canvas.DrawLine(Vector2{ dotX - 5, dotY }, Vector2{ dotX + 5, dotY });
        canvas.DrawLine(Vector2{ dotX, dotY - 5 }, Vector2{ dotX, dotY + 5 });

        canvas.SetColor((char)200, (char)200, (char)200, (char)255);
        canvas.SetPosition(Vector2{ 50, 265 });
        canvas.DrawString("Stick Vector", 0.9f, 0.9f);
    }
}

void Plugin::RenderSettings() {
    if (ImGui::Checkbox("Enable Trainer", &enabled)) {
        target.active = false;
    }

    ImGui::Separator();
    const char* modes[] = { "Inverted Space Sim (Relative Physics)", "Free Flight Gates (Fly Around Pitch)" };
    if (ImGui::Combo("Training Mode", &currentMode, modes, IM_ARRAYSIZE(modes))) {
        target.active = false;
    }

    if (currentMode == static_cast<int>(TrainerMode::InvertedRelativeSim)) {
        ImGui::SliderFloat("Spawn Distance", &relativeSpawnDistance, 800.0f, 2500.0f, "%.0f");
    }

    ImGui::Separator();
    const char* rollModes[] = { "Any Roll (Nose Hit Only)", "Require Cardinal Phases (0/90/180/270)" };
    ImGui::Combo("Roll Phase Condition", &rollPhaseMode, rollModes, IM_ARRAYSIZE(rollModes));
    if (rollPhaseMode == static_cast<int>(RollPhaseMode::CardinalPhases)) {
        ImGui::SliderFloat("Roll Tolerance", &rollToleranceDegrees, 15.0f, 45.0f, "%.0f deg");
    }

    ImGui::SliderFloat("Target Hit Radius", &targetHitRadius, 100.0f, 300.0f, "%.0f uu");
    ImGui::Checkbox("Show Analog Stick Guide HUD", &showStickGuide);

    ImGui::Separator();
    ImGui::Text("Anime Visual Effects:");
    ImGui::Checkbox("Enable Anime Speed Lines", &showAnimeSpeedLines);
    if (showAnimeSpeedLines) {
        ImGui::SliderInt("Line Density", &maxAnimeLines, 15, 80);
        ImGui::SliderFloat("Line Length", &animeLineLengthBase, 150.0f, 600.0f, "%.0f");
        ImGui::SliderFloat("Line Thickness", &animeLineThickness, 1.0f, 5.0f, "%.1f");
    }

    ImGui::Separator();
    if (ImGui::SliderFloat("Game Speed (Slow-Mo)", &gameSpeed, 0.2f, 1.0f, "%.2f")) {
        if (_globalCvarManager) {
            auto speedCvar = _globalCvarManager->getCvar("sv_soccar_gamespeed");
            if (speedCvar) speedCvar.setValue(gameSpeed);
        }
    }

    if (rollPhaseMode == static_cast<int>(RollPhaseMode::CardinalPhases)) {
        ImGui::Separator();
        ImGui::Text("Phase Performance Metrics (Average Time to Intercept):");
        const char* phaseLabels[] = { "Upright (Hood):", "Right Flank:", "Inverted (Belly):", "Left Flank:" };
        for (int i = 0; i < 4; ++i) {
            float avg = (phaseHitCounts[i] > 0) ? (phaseTotalTimeMs[i] / static_cast<float>(phaseHitCounts[i])) : 0.0f;
            ImGui::Text("%s %.0f ms (%d hits)", phaseLabels[i], avg, phaseHitCounts[i]);
        }
        if (ImGui::Button("Reset Phase Metrics")) {
            phaseHitCounts.fill(0);
            phaseTotalTimeMs.fill(0.0f);
        }
    }
}