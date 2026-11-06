#include "pch.h"
#include "Plugin.h"
#include <cmath>
#include <algorithm>

BAKKESMOD_PLUGIN(Plugin, "Car Control Orientation Trainer", "1.0", PLUGINTYPE_FREEPLAY)

std::shared_ptr<CVarManagerWrapper> _globalCvarManager;

constexpr float PI_F = 3.14159265358979323846f;
constexpr float RAD_TO_DEG = 180.0f / PI_F;
constexpr float UROT_TO_RAD = PI_F / 32768.0f;
constexpr float BALL_RADIUS = 93.0f;
constexpr float MIN_FLOOR_CLEARANCE = 30.0f;

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

static void DrawThickLineIn3D(CanvasWrapper& canvas, const Vector& start, const Vector& end, float thickness = 3.0f) {
    Vector2 p1 = canvas.Project(start);
    Vector2 p2 = canvas.Project(end);

    if (p1.X <= 0 && p1.Y <= 0 && p2.X <= 0 && p2.Y <= 0) return;

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

        Vector2 lineStart = { p1.X + offX, p1.Y + offY };
        Vector2 lineEnd   = { p2.X + offX, p2.Y + offY };
        canvas.DrawLine(lineStart, lineEnd);
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
        std::bind(&Plugin::OnPhysicsTick, this, std::placeholders::_1)
    );

    gameWrapper->RegisterDrawable(std::bind(&Plugin::Render, this, std::placeholders::_1));
}

void Plugin::onUnload() {
    gameWrapper->UnhookEvent("Function TAGame.Car_TA.SetVehicleInput");
}

void Plugin::OnPhysicsTick(std::string eventName) {
    if (!enabled || !gameWrapper->IsInFreeplay()) return;

    ServerWrapper server = gameWrapper->GetGameEventAsServer();
    if (server.IsNull()) return;

    CarWrapper car = gameWrapper->GetLocalCar();
    if (car.IsNull()) return;

    anchorPosition.Z = anchorZ;
    car.SetLocation(anchorPosition);
    car.SetVelocity(Vector{ 0.f, 0.f, 0.f });

    if (!currentTarget.hasTarget || !initializedTarget) {
        GenerateNewTarget(car);
        initializedTarget = true;
        return;
    }

    BallWrapper ball = server.GetBall();
    if (!ball.IsNull()) {
        ball.SetLocation(currentTarget.ballPosition);
        ball.SetVelocity(Vector{ 0.f, 0.f, 0.f });
        ball.SetAngularVelocity(Vector{ 0.f, 0.f, 0.f }, false);
    }

    constexpr float deltaTime = 1.0f / 120.0f;
    if (CheckAlignment(car, deltaTime)) {
        auto now = std::chrono::steady_clock::now();
        lastReactionTimeMs = std::chrono::duration<float, std::milli>(now - targetStartTime).count();
        targetsCompleted++;
        GenerateNewTarget(car);
    }
}

void Plugin::GenerateNewTarget(CarWrapper car) {
    float minAllowedZ = BALL_RADIUS + MIN_FLOOR_CLEARANCE;
    float maxDownwardDrop = anchorZ - minAllowedZ;
    float maxSinPitch = std::clamp(maxDownwardDrop / targetDistance, 0.1f, 1.0f);
    float maxDownwardPitchDeg = std::asin(maxSinPitch) * RAD_TO_DEG;

    int minPitchUnits = -static_cast<int>((maxDownwardPitchDeg / 180.0f) * 32768.0f);
    int maxPitchUnits = 14000;

    int pitchRange = maxPitchUnits - minPitchUnits;
    int pitch = minPitchUnits + (rand() % (pitchRange > 0 ? pitchRange : 1));
    int yaw   = (rand() % 65536) - 32768;
    int roll  = requireRollAlignment ? ((rand() % 65536) - 32768) : 0;

    currentTarget.targetRotator = Rotator(pitch, yaw, roll);
    Vector dummyRight;
    GetRotatorAxes(currentTarget.targetRotator, currentTarget.targetForward, dummyRight, currentTarget.targetUp);

    Vector fwdNorm = VectorNormalize(currentTarget.targetForward);
    float rawZ = anchorPosition.Z + (fwdNorm.Z * targetDistance);
    float safeZ = std::max(rawZ, minAllowedZ);

    currentTarget.ballPosition = Vector{
        anchorPosition.X + (fwdNorm.X * targetDistance),
        anchorPosition.Y + (fwdNorm.Y * targetDistance),
        safeZ
    };

    currentTarget.hasTarget = true;
    currentHoldTime = 0.0f;
    targetStartTime = std::chrono::steady_clock::now();
}

bool Plugin::CheckAlignment(CarWrapper car, float deltaTime) {
    if (!currentTarget.hasTarget || car.IsNull()) return false;

    Rotator carRot = car.GetRotation();
    Vector carForward, carRight, carUp;
    GetRotatorAxes(carRot, carForward, carRight, carUp);

    Vector normCarForward = VectorNormalize(carForward);
    Vector normTargetForward = VectorNormalize(currentTarget.targetForward);
    float forwardDot = std::clamp(VectorDot(normCarForward, normTargetForward), -1.0f, 1.0f);

    currentNoseErrorDeg = std::acos(forwardDot) * RAD_TO_DEG;
    bool noseAligned = (currentNoseErrorDeg <= noseToleranceDegrees);

    bool rollAligned = true;
    if (requireRollAlignment) {
        Vector normCarUp = VectorNormalize(carUp);
        Vector normTargetUp = VectorNormalize(currentTarget.targetUp);
        float upDot = std::clamp(VectorDot(normCarUp, normTargetUp), -1.0f, 1.0f);
        currentRollErrorDeg = std::acos(upDot) * RAD_TO_DEG;
        rollAligned = (currentRollErrorDeg <= rollToleranceDegrees);
    } else {
        currentRollErrorDeg = 0.0f;
    }

    if (noseAligned && rollAligned) {
        currentHoldTime += deltaTime;
        if (currentHoldTime >= holdDuration) return true;
    } else {
        currentHoldTime = std::max(0.0f, currentHoldTime - (deltaTime * 1.5f));
    }

    return false;
}

void Plugin::Render(CanvasWrapper canvas) {
    if (!enabled || !gameWrapper->IsInFreeplay()) return;

    // Telemetry HUD
    canvas.SetColor((char)255, (char)255, (char)255, (char)255);
    canvas.SetPosition(Vector2{ 40, 60 });
    canvas.DrawString("Car Control Trainer: ACTIVE (Ball Cam)", 1.3f, 1.3f);

    canvas.SetPosition(Vector2{ 40, 85 });
    canvas.DrawString("Score: " + std::to_string(targetsCompleted) + "  |  Last: " + std::to_string(static_cast<int>(lastReactionTimeMs)) + " ms", 1.2f, 1.2f);

    if (currentNoseErrorDeg <= noseToleranceDegrees) {
        canvas.SetColor((char)0, (char)255, (char)0, (char)255);
    } else {
        canvas.SetColor((char)255, (char)180, (char)180, (char)255);
    }
    canvas.SetPosition(Vector2{ 40, 110 });
    canvas.DrawString("Nose Error: " + std::to_string(static_cast<int>(currentNoseErrorDeg)) + " deg (Tol: " + std::to_string(static_cast<int>(noseToleranceDegrees)) + " deg)", 1.2f, 1.2f);

    if (requireRollAlignment) {
        if (currentRollErrorDeg <= rollToleranceDegrees) {
            canvas.SetColor((char)0, (char)255, (char)0, (char)255);
        } else {
            canvas.SetColor((char)255, (char)180, (char)180, (char)255);
        }
        canvas.SetPosition(Vector2{ 40, 135 });
        canvas.DrawString("Roll Error: " + std::to_string(static_cast<int>(currentRollErrorDeg)) + " deg (Tol: " + std::to_string(static_cast<int>(rollToleranceDegrees)) + " deg)", 1.2f, 1.2f);
    }

    canvas.SetColor((char)255, (char)255, (char)255, (char)255);
    canvas.SetPosition(Vector2{ 40, requireRollAlignment ? 160 : 135 });
    canvas.DrawString("Hold Lock: " + std::to_string(static_cast<int>((currentHoldTime / holdDuration) * 100)) + "%", 1.2f, 1.2f);

    // Thick, compact DAR roof arrow
    if (requireRollAlignment && currentTarget.hasTarget) {
        CarWrapper car = gameWrapper->GetLocalCar();
        if (!car.IsNull()) {
            Vector carPos = car.GetLocation();
            Vector up = VectorNormalize(currentTarget.targetUp);
            Vector fwd = VectorNormalize(currentTarget.targetForward);
            Vector right = VectorNormalize(VectorCross(fwd, up));

            // Color turns green when roll is within tolerance
            if (currentRollErrorDeg <= rollToleranceDegrees) {
                canvas.SetColor((char)0, (char)255, (char)0, (char)255);
            } else {
                canvas.SetColor((char)0, (char)220, (char)255, (char)255); // Cyan
            }

            // Arrow shaft (short: extends from roof at +35 units up to +125 units)
            Vector shaftStart = Vector{ carPos.X + (up.X * 35.0f), carPos.Y + (up.Y * 35.0f), carPos.Z + (up.Z * 35.0f) };
            Vector arrowTip   = Vector{ carPos.X + (up.X * 125.0f), carPos.Y + (up.Y * 125.0f), carPos.Z + (up.Z * 125.0f) };
            DrawThickLineIn3D(canvas, shaftStart, arrowTip, 6.0f);

            // Arrowhead fins
            Vector barbLeft = Vector{
                arrowTip.X - (up.X * 28.0f) + (right.X * 22.0f),
                arrowTip.Y - (up.Y * 28.0f) + (right.Y * 22.0f),
                arrowTip.Z - (up.Z * 28.0f) + (right.Z * 22.0f)
            };
            Vector barbRight = Vector{
                arrowTip.X - (up.X * 28.0f) - (right.X * 22.0f),
                arrowTip.Y - (up.Y * 28.0f) - (right.Y * 22.0f),
                arrowTip.Z - (up.Z * 28.0f) - (right.Z * 22.0f)
            };
            DrawThickLineIn3D(canvas, arrowTip, barbLeft, 5.0f);
            DrawThickLineIn3D(canvas, arrowTip, barbRight, 5.0f);
        }
    }
}

void Plugin::RenderSettings() {
    if (ImGui::Checkbox("Enable Trainer", &enabled)) {
        initializedTarget = false;
        currentTarget.hasTarget = false;
        currentHoldTime = 0.0f;
    }

    ImGui::Separator();
    ImGui::Checkbox("Require Roll Alignment (DAR)", &requireRollAlignment);
    ImGui::SliderFloat("Nose Tolerance (degrees)", &noseToleranceDegrees, 8.0f, 35.0f, "%.1f deg");
    if (requireRollAlignment) {
        ImGui::SliderFloat("Roll Tolerance (degrees)", &rollToleranceDegrees, 10.0f, 45.0f, "%.1f deg");
    }
    ImGui::SliderFloat("Hold Duration (sec)", &holdDuration, 0.05f, 0.40f, "%.2f s");
    ImGui::SliderFloat("Anchor Height (Z)", &anchorZ, 400.0f, 1500.0f, "%.0f");
    ImGui::SliderFloat("Target Ball Distance", &targetDistance, 400.0f, 1500.0f, "%.0f");
}