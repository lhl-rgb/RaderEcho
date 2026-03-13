#pragma once

#include <complex>
#include <vector>
#include <string>
#include <memory>
#include <array>
#include <cmath>
#include <random>

// ============================================================================
// 基本类型定义
// ============================================================================

#define USE_SINGLE_PRECISION
#ifdef USE_SINGLE_PRECISION
using SignalType = float;
using Complex = std::complex<SignalType>;
using ComplexVector = std::vector<Complex>;
using ComplexMatrix = std::vector<ComplexVector>;
static constexpr SignalType C_LIGHT = 3e8f;
static constexpr SignalType PI_VAL = 3.14159265358979323846f;
static constexpr SignalType K_BOLTZMANN = 1.38e-23f;
static constexpr SignalType T0 = 290.0f;
#else
using SignalType = double;
using Complex = std::complex<SignalType>;
using ComplexVector = std::vector<Complex>;
using ComplexMatrix = std::vector<ComplexVector>;
static constexpr SignalType C_LIGHT = 3e8;
static constexpr SignalType PI_VAL = 3.14159265358979323846;
static constexpr SignalType K_BOLTZMANN = 1.38e-23;
static constexpr SignalType T0 = 290.0;
#endif

// ============================================================================
// 坐标和位置相关
// ============================================================================

struct Position3D {
    SignalType x = 0, y = 0, z = 0;

    Position3D() = default;
    Position3D(SignalType x, SignalType y, SignalType z) : x(x), y(y), z(z) {}

    SignalType distanceTo(const Position3D& other) const {
        SignalType dx = x - other.x;
        SignalType dy = y - other.y;
        SignalType dz = z - other.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    SignalType range() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    SignalType azimuth() const {
        return std::atan2(y, x);
    }

    SignalType elevation() const {
        SignalType r = std::sqrt(x * x + y * y);
        return std::atan2(z, r);
    }
};

struct Velocity3D {
    SignalType vx = 0, vy = 0, vz = 0;

    Velocity3D() = default;
    Velocity3D(SignalType vx, SignalType vy, SignalType vz) : vx(vx), vy(vy), vz(vz) {}

    SignalType speed() const {
        return std::sqrt(vx * vx + vy * vy + vz * vz);
    }

    SignalType radialVelocity(const Position3D& pos) const {
        SignalType r = pos.range();
        if (r < 1e-6) return 0;
        return (pos.x * vx + pos.y * vy + pos.z * vz) / r;
    }
};

struct Acceleration3D {
    SignalType ax = 0, ay = 0, az = 0;

    Acceleration3D() = default;
    Acceleration3D(SignalType ax, SignalType ay, SignalType az) : ax(ax), ay(ay), az(az) {}

    SignalType magnitude() const {
        return std::sqrt(ax * ax + ay * ay + az * az);
    }
};

// ============================================================================
// 网格单元定义（用于杂波计算）
// ============================================================================

struct ClutterCell {
    size_t rangeIndex = 0;
    size_t azimuthIndex = 0;
    size_t elevationIndex = 0;

    SignalType range = 0;
    SignalType azimuth = 0;
    SignalType elevation = 0;

    SignalType rangeSpan = 0;
    SignalType azimuthSpan = 0;
    SignalType elevationSpan = 0;

    SignalType antennaGain = 0;
    SignalType clutterPower = 0;

    size_t sequenceIndex = 0;
};

// ============================================================================
// 波束指向定义
// ============================================================================

struct BeamPosition {
    SignalType azimuth = 0;
    SignalType elevation = 0;
    int beamCode = 0;

    BeamPosition() = default;
    BeamPosition(SignalType az, SignalType el, int code)
        : azimuth(az), elevation(el), beamCode(code) {}
};

// ============================================================================
// 雷达波形参数
// ============================================================================

enum class WaveformType {
    LFM = 0,
    NLFM = 1,
    PhaseCoded = 2
};

enum class ModulationType {
    UpChirp = 0,
    DownChirp = 1,
    Triangle = 2
};

struct WaveformParams {
    WaveformType type = WaveformType::LFM;
    ModulationType modulation = ModulationType::UpChirp;

    SignalType bandwidth = 0;
    SignalType pulseWidth = 0;
    SignalType sampleInterval = 0;

    SignalType chirpRate() const {
        return bandwidth / pulseWidth;
    }

    SignalType timeBandwidthProduct() const {
        return bandwidth * pulseWidth;
    }
};

// ============================================================================
// 天线参数
// ============================================================================

struct AntennaParams {
    SignalType gain_dB = 0;
    SignalType height = 0;
    SignalType azBeamwidth_deg = 0;
    SignalType elBeamwidth_deg = 0;

    int numElementsAz = 1;
    int numElementsEl = 1;
    SignalType elementSpacing = 0;

    SignalType scanRate = 0;
    std::string polarization = "HH";

    SignalType gain_linear = 1;
    SignalType azBeamwidth_rad = 0;
    SignalType elBeamwidth_rad = 0;
    SignalType wavelength = 0;

    void updateDerivedParams(SignalType frequency) {
        gain_linear = std::pow(10.0, gain_dB / 10.0);
        azBeamwidth_rad = azBeamwidth_deg * PI_VAL / 180.0;
        elBeamwidth_rad = elBeamwidth_deg * PI_VAL / 180.0;
        wavelength = C_LIGHT / frequency;
        if (elementSpacing == 0) {
            elementSpacing = wavelength / 2.0;
        }
    }

    SignalType calculateArrayGain(SignalType azAngle, SignalType elAngle,
                                   SignalType azSteer, SignalType elSteer) const {
        SignalType k = 2 * PI_VAL / wavelength;

        SignalType azGain = 1;
        if (numElementsAz > 1) {
            SignalType sinAz = std::sin(azAngle) - std::sin(azSteer);
            SignalType psi = k * elementSpacing * sinAz;
            if (std::abs(psi) > 1e-10) {
                azGain = std::abs(std::sin(numElementsAz * psi / 2) /
                         (numElementsAz * std::sin(psi / 2)));
            }
            azGain = std::pow(azGain, 2);
        }

        SignalType elGain = 1;
        if (numElementsEl > 1) {
            SignalType sinEl = std::sin(elAngle) - std::sin(elSteer);
            SignalType psi = k * elementSpacing * sinEl;
            if (std::abs(psi) > 1e-10) {
                elGain = std::abs(std::sin(numElementsEl * psi / 2) /
                         (numElementsEl * std::sin(psi / 2)));
            }
            elGain = std::pow(elGain, 2);
        }

        return azGain * elGain;
    }
};

// ============================================================================
// 雷达系统参数
// ============================================================================

struct RadarSystemParams {
    SignalType frequency = 0;
    SignalType bandwidth = 0;
    SignalType prf = 0;
    SignalType pulseWidth = 0;
    SignalType sampleRate = 0;
    SignalType peakPower = 0;

    SignalType noiseFigure = 0;
    SignalType loss = 0;

    SignalType maxRange = 0;
    SignalType minRange = 0;

    int numPulsesPerCPI = 0;
    SignalType cpiDuration = 0;

    SignalType wavelength = 0;
    SignalType pri = 0;
    SignalType rangeResolution = 0;
    int numRangeBins = 0;
    SignalType rangeBinSize = 0;

    void updateDerivedParams() {
        wavelength = C_LIGHT / frequency;
        pri = 1.0 / prf;
        cpiDuration = numPulsesPerCPI / prf;
        rangeResolution = C_LIGHT / (2 * bandwidth);
        minRange = C_LIGHT * pulseWidth / 2.0;
        rangeBinSize = C_LIGHT / (2 * sampleRate);
        numRangeBins = static_cast<int>((maxRange - minRange) / rangeBinSize);
    }
};

// ============================================================================
// 杂波参数
// ============================================================================

enum class ClutterDistributionType {
    Rayleigh = 0,
    Weibull = 1,
    LogNormal = 2,
    K_Distribution = 3
};

struct SpectrumParams {
    SignalType centerFreq = 0;
    SignalType bandwidth = 0;
    SignalType shape = 0;
};

struct ClutterCellParams {
    SignalType sigma0 = 0;
    SignalType grazingAngle = 0;
    SignalType rmsHeight = 0;
    SignalType windSpeed = 0;
};

struct ClutterParams {
    ClutterDistributionType type = ClutterDistributionType::Rayleigh;
    SpectrumParams spectrum;
    ClutterCellParams cellParams;

    SignalType weibullScale = 1;
    SignalType weibullShape = 1;
    SignalType logNormalMean = 0;
    SignalType logNormalStd = 1;
    SignalType kDistShape = 1;
};

// ============================================================================
// 目标参数
// ============================================================================

enum class SwerlingModel {
    Swerling0 = 0,
    Swerling1 = 1,
    Swerling2 = 2,
    Swerling3 = 3,
    Swerling4 = 4
};

enum class MotionModel {
    Stationary = 0,
    ConstantVelocity = 1,
    ConstantAcceleration = 2,
    VariableAcceleration = 3,
    Circular = 4,
    SineWave = 5
};

struct TargetParams {
    size_t id = 0;
    Position3D position;
    Velocity3D velocity;
    Acceleration3D acceleration;

    MotionModel motionModel = MotionModel::ConstantVelocity;
    SwerlingModel swerlingModel = SwerlingModel::Swerling1;

    SignalType rcs_mean = 1;
    SignalType currentRCS = 1;

    SignalType turnRate = 0;
    SignalType turnRadius = 0;

    SignalType sineAmplitude = 0;
    SignalType sineFrequency = 0;
};

// ============================================================================
// 仿真配置
// ============================================================================

struct SimulationConfig {
    std::string name = "Simulation";
    SignalType duration = 0;
    SignalType startTime = 0;

    bool generateClutter = true;
    bool generateTargets = true;
    bool addNoise = true;

    std::string beamCodeFile = "";
    std::string outputFile = "";
    std::string clutterDataFile = "";

    size_t clutterPoolSize = 100;
    size_t clutterSequenceLength = 0;
};

// ============================================================================
// 辅助函数
// ============================================================================

inline SignalType deg2rad(SignalType deg) {
    return deg * PI_VAL / 180.0;
}

inline SignalType rad2deg(SignalType rad) {
    return rad * 180.0 / PI_VAL;
}

inline SignalType dB2linear(SignalType dB) {
    return std::pow(10.0, dB / 10.0);
}

inline SignalType linear2dB(SignalType linear) {
    return 10.0 * std::log10(linear);
}
