#include "TargetManager.h"
#include "Logger.h"
#include <random>
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void StationaryModel::updatePosition(TargetParams& target, SignalType dt) {
    (void)target;
    (void)dt;
}

void ConstantVelocityModel::updatePosition(TargetParams& target, SignalType dt) {
    target.position.x += target.velocity.vx * dt;
    target.position.y += target.velocity.vy * dt;
    target.position.z += target.velocity.vz * dt;
}

void ConstantAccelerationModel::updatePosition(TargetParams& target, SignalType dt) {
    target.velocity.vx += target.acceleration.ax * dt;
    target.velocity.vy += target.acceleration.ay * dt;
    target.velocity.vz += target.acceleration.az * dt;

    target.position.x += target.velocity.vx * dt + 0.5 * target.acceleration.ax * dt * dt;
    target.position.y += target.velocity.vy * dt + 0.5 * target.acceleration.ay * dt * dt;
    target.position.z += target.velocity.vz * dt + 0.5 * target.acceleration.az * dt * dt;
}

VariableAccelerationModel::VariableAccelerationModel(
    std::function<Acceleration3D(SignalType t)> accelFunc)
    : m_accelFunc(accelFunc) {}

void VariableAccelerationModel::updatePosition(TargetParams& target, SignalType dt) {
    Acceleration3D accel = m_accelFunc(target.position.range() / C_LIGHT);

    target.velocity.vx += accel.ax * dt;
    target.velocity.vy += accel.ay * dt;
    target.velocity.vz += accel.az * dt;

    target.position.x += target.velocity.vx * dt + 0.5 * accel.ax * dt * dt;
    target.position.y += target.velocity.vy * dt + 0.5 * accel.ay * dt * dt;
    target.position.z += target.velocity.vz * dt + 0.5 * accel.az * dt * dt;
}

void CircularModel::updatePosition(TargetParams& target, SignalType dt) {
    SignalType omega = target.turnRate;
    SignalType speed = std::abs(omega) * target.turnRadius;

    SignalType cosTheta = std::cos(omega * dt);
    SignalType sinTheta = std::sin(omega * dt);

    SignalType relX = target.position.x;
    SignalType relY = target.position.y;

    target.position.x = relX * cosTheta - relY * sinTheta;
    target.position.y = relX * sinTheta + relY * cosTheta;

    SignalType currentAngle = std::atan2(target.position.y, target.position.x);
    target.velocity.vx = -speed * std::sin(currentAngle);
    target.velocity.vy = speed * std::cos(currentAngle);
}

void SineWaveModel::updatePosition(TargetParams& target, SignalType dt) {
    SignalType t = target.position.range() / C_LIGHT;
    SignalType dv = target.sineAmplitude * std::sin(2 * M_PI * target.sineFrequency * t);

    target.position.x += (target.velocity.vx + dv) * dt;
    target.position.y += (target.velocity.vy + dv) * dt;
    target.position.z += (target.velocity.vz + dv) * dt;
}

TargetManager::TargetManager() {
    m_models[MotionModel::Stationary] = std::make_unique<StationaryModel>();
    m_models[MotionModel::ConstantVelocity] = std::make_unique<ConstantVelocityModel>();
    m_models[MotionModel::ConstantAcceleration] = std::make_unique<ConstantAccelerationModel>();
    m_models[MotionModel::Circular] = std::make_unique<CircularModel>();
    m_models[MotionModel::SineWave] = std::make_unique<SineWaveModel>();
}

TargetManager::~TargetManager() = default;

size_t TargetManager::addTarget(const TargetParams& target) {
    TargetParams newTarget = target;
    newTarget.id = m_nextId++;
    m_targets.push_back(newTarget);
    LOG_INFO("Added target ID: " + std::to_string(newTarget.id));
    return newTarget.id;
}

void TargetManager::removeTarget(size_t id) {
    auto it = std::find_if(m_targets.begin(), m_targets.end(),
                           [id](const TargetParams& t) { return t.id == id; });
    if (it != m_targets.end()) {
        m_targets.erase(it);
        LOG_INFO("Removed target ID: " + std::to_string(id));
    }
}

void TargetManager::updateTarget(size_t id, const TargetParams& target) {
    auto it = std::find_if(m_targets.begin(), m_targets.end(),
                           [id](const TargetParams& t) { return t.id == id; });
    if (it != m_targets.end()) {
        *it = target;
        LOG_INFO("Updated target ID: " + std::to_string(id));
    }
}

TargetParams* TargetManager::getTarget(size_t id) {
    auto it = std::find_if(m_targets.begin(), m_targets.end(),
                           [id](const TargetParams& t) { return t.id == id; });
    return (it != m_targets.end()) ? &(*it) : nullptr;
}

std::vector<TargetParams> TargetManager::getAllTargets() const {
    return m_targets;
}

void TargetManager::updateAllTargets(SignalType dt, SignalType currentTime) {
    for (auto& target : m_targets) {
        (void)currentTime;
        auto it = m_models.find(target.motionModel);
        if (it != m_models.end() && it->second) {
            it->second->updatePosition(target, dt);
        }
    }
}

void TargetManager::updateTargetRCS(TargetParams& target, bool cpiStart) {
    std::random_device rd;
    static std::mt19937 gen(rd());
    std::exponential_distribution<SignalType> expDist(1.0);

    switch (target.swerlingModel) {
        case SwerlingModel::Swerling0:
            target.currentRCS = target.rcs_mean;
            break;

        case SwerlingModel::Swerling1:
            if (cpiStart) {
                SignalType u = static_cast<SignalType>(gen()) / gen.max();
                target.currentRCS = -target.rcs_mean * std::log(1.0 - u);
            }
            break;

        case SwerlingModel::Swerling2:
            {
                SignalType u = static_cast<SignalType>(gen()) / gen.max();
                target.currentRCS = -target.rcs_mean * std::log(1.0 - u);
            }
            break;

        case SwerlingModel::Swerling3:
            if (cpiStart) {
                SignalType x1 = expDist(gen);
                SignalType x2 = expDist(gen);
                target.currentRCS = target.rcs_mean * (x1 + x2) / 2.0;
            }
            break;

        case SwerlingModel::Swerling4:
            {
                SignalType x1 = expDist(gen);
                SignalType x2 = expDist(gen);
                target.currentRCS = target.rcs_mean * (x1 + x2) / 2.0;
            }
            break;
    }
}

void TargetManager::setMotionModel(size_t id, MotionModel model) {
    auto it = std::find_if(m_targets.begin(), m_targets.end(),
                           [id](const TargetParams& t) { return t.id == id; });
    if (it != m_targets.end()) {
        it->motionModel = model;
    }
}

void TargetManager::clear() {
    m_targets.clear();
    m_nextId = 1;
    LOG_INFO("All targets cleared");
}
