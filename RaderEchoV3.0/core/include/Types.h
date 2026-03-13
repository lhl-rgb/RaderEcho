#pragma once

#include <complex>
#include <vector>
#include <string>
#include <memory>
#include <array>

// ============================================================================
// 基本类型定义
// ============================================================================

#define USE_SINGLE_PRECISION
#ifdef USE_SINGLE_PRECISION
using SignalType = float;
using Complex = std::complex<SignalType>;
using ComplexVector = std::vector<Complex>;
using ComplexMatrix = std::vector<ComplexVector>;
static constexpr SignalType C_LIGHT = 3e8;
static constexpr SignalType PI = 3.14159265358979323846;
static constexpr SignalType K_BOLTZMANN = 1.38e-23;
static constexpr SignalType T0 = 290.0;
#else
using SignalType = double;
using Complex = std::complex<SignalType>;
using ComplexVector = std::vector<Complex>;
using ComplexMatrix = std::vector<ComplexVector>;
static constexpr SignalType C_LIGHT = 3e8;
static constexpr SignalType PI = 3.14159265358979323846;
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
    size_t rangeIndex = 0;       // 距离索引
    size_t azimuthIndex = 0;     // 方位索引
    size_t elevationIndex = 0;   // 俯仰索引

    SignalType range = 0;        // 斜距 (m)
    SignalType azimuth = 0;      // 方位角 (rad)
    SignalType elevation = 0;    // 俯仰角 (rad)

    SignalType rangeSpan = 0;    // 距离跨度 (m)
    SignalType azimuthSpan = 0;  // 方位跨度 (rad)
    SignalType elevationSpan = 0;// 俯仰跨度 (rad)

    SignalType antennaGain = 0;  // 天线增益（线性值）
    SignalType clutterPower = 0; // 杂波功率

    // 杂波序列索引（用于从内存池中获取）
    size_t sequenceIndex = 0;
};

// ============================================================================
// 波束指向定义
// ============================================================================

struct BeamPosition {
    SignalType azimuth = 0;      // 方位指向 (deg)
    SignalType elevation = 0;    // 俯仰指向 (deg)
    int beamCode = 0;            // 波位码

    BeamPosition() = default;
    BeamPosition(SignalType az, SignalType el, int code)
        : azimuth(az), elevation(el), beamCode(code) {}
};

// ============================================================================
// 雷达波形参数
// ============================================================================

enum class WaveformType {
    LFM = 0,      // 线性调频
    NLFM = 1,     // 非线性调频
    PhaseCoded = 2// 相位编码
};

enum class ModulationType {
    UpChirp = 0,
    DownChirp = 1,
    Triangle = 2
};

struct WaveformParams {
    WaveformType type = WaveformType::LFM;
    ModulationType modulation = ModulationType::UpChirp;

    SignalType bandwidth = 0;    // 带宽 (Hz)
    SignalType pulseWidth = 0;   // 脉宽 (s)
    SignalType sampleInterval = 0;          // 采样间隔 (s)

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
    // 基本参数
    SignalType gain_dB = 0;          // 天线增益 (dB)
    SignalType height = 0;           // 天线高度 (m)
    SignalType azBeamwidth_deg = 0;  // 水平波束宽度 (deg)
    SignalType elBeamwidth_deg = 0;  // 俯仰波束宽度 (deg)

    // 相控阵参数
    int numElementsAz = 1;           // 方位阵元数
    int numElementsEl = 1;           // 俯仰阵元数
    SignalType elementSpacing = 0;   // 阵元间距 (m)

    // 扫描参数
    SignalType scanRate = 0;         // 扫描速率 (deg/s)
    std::string polarization = "HH"; // 极化方式

    // 计算参数
    SignalType gain_linear = 1;      // 线性增益
    SignalType azBeamwidth_rad = 0;  // 水平波束宽度 (rad)
    SignalType elBeamwidth_rad = 0;  // 俯仰波束宽度 (rad)
    SignalType wavelength = 0;       // 波长 (m)

    void updateDerivedParams(SignalType frequency) {
        gain_linear = std::pow(10.0, gain_dB / 10.0);
        azBeamwidth_rad = azBeamwidth_deg * PI / 180.0;
        elBeamwidth_rad = elBeamwidth_deg * PI / 180.0;
        wavelength = C_LIGHT / frequency;
        if (elementSpacing == 0) {
            elementSpacing = wavelength / 2.0;
        }
    }

    // 计算阵列方向图增益
    SignalType calculateArrayGain(SignalType azAngle, SignalType elAngle,
                                   SignalType azSteer, SignalType elSteer) const {
        SignalType k = 2 * PI / wavelength;

        // 方位方向图
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

        // 俯仰方向图
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
    // 基本参数
    SignalType frequency = 0;        // 载频 (Hz)
    SignalType bandwidth = 0;        // 信号带宽 (Hz)
    SignalType prf = 0;              // 脉冲重复频率 (Hz)
    SignalType pulseWidth = 0;       // 脉宽 (s)
    SignalType sampleRate = 0;       // 采样率 (Hz)
    SignalType peakPower = 0;        // 峰值功率 (W)

    // 系统参数
    SignalType noiseFigure = 0;      // 噪声系数 (dB)
    SignalType loss = 0;             // 系统损耗 (dB)

    // 观测参数
    SignalType maxRange = 0;         // 最大观测距离 (m)
    SignalType minRange = 0;         // 最小观测距离 (m)

    // CPI 参数
    int numPulsesPerCPI = 0;         // 每个 CPI 的脉冲数
    SignalType cpiDuration = 0;      // CPI 持续时间 (s)

    // 推导参数
    SignalType wavelength = 0;       // 波长 (m)
    SignalType pri = 0;              // 脉冲重复间隔 (s)
    SignalType rangeResolution = 0;  // 距离分辨率 (m)
    int numRangeBins = 0;            // 距离门数量
    SignalType rangeBinSize = 0;     // 距离门大小 (m)

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
    SignalType centerFreq = 0;       // 杂波谱中心频率 (Hz)
    SignalType bandwidth = 0;        // 杂波谱带宽 (Hz)
    SignalType shape = 0;            // 谱形状参数
};

struct ClutterCellParams {
    SignalType sigma0 = 0;           // 后向散射系数
    SignalType grazingAngle = 0;     // 掠射角 (rad)
    SignalType rmsHeight = 0;        // 海面 RMS 高度 (m)
    SignalType windSpeed = 0;        // 风速 (m/s)
};

struct ClutterParams {
    ClutterDistributionType type = ClutterDistributionType::Rayleigh;
    SpectrumParams spectrum;
    ClutterCellParams cellParams;

    // 分布参数
    SignalType weibullScale = 1;     // Weibull 尺度参数
    SignalType weibullShape = 1;     // Weibull 形状参数
    SignalType logNormalMean = 0;    // LogNormal 均值
    SignalType logNormalStd = 1;     // LogNormal 标准差
    SignalType kDistShape = 1;       // K 分布形状参数
};

// ============================================================================
// 目标参数
// ============================================================================

enum class SwerlingModel {
    Swerling0 = 0,   // 非起伏
    Swerling1 = 1,   // 慢起伏 Rayleigh
    Swerling2 = 2,   // 快起伏 Rayleigh
    Swerling3 = 3,   // 慢起伏 Chi-square
    Swerling4 = 4    // 快起伏 Chi-square
};

enum class MotionModel {
    Stationary = 0,  // 静止
    ConstantVelocity = 1,  // 匀速
    ConstantAcceleration = 2,  // 匀加速
    VariableAcceleration = 3,  // 变加速
    Circular = 4,    // 圆周运动
    SineWave = 5     // 正弦机动
};

struct TargetParams {
    size_t id = 0;
    Position3D position;
    Velocity3D velocity;
    Acceleration3D acceleration;

    MotionModel motionModel = MotionModel::ConstantVelocity;
    SwerlingModel swerlingModel = SwerlingModel::Swerling1;

    SignalType rcs_mean = 1;       // 平均 RCS (m^2)
    SignalType currentRCS = 1;     // 当前 RCS

    // 圆周运动参数
    SignalType turnRate = 0;       // 转弯速率 (rad/s)
    SignalType turnRadius = 0;     // 转弯半径 (m)

    // 正弦机动参数
    SignalType sineAmplitude = 0;  // 正弦幅度
    SignalType sineFrequency = 0;  // 正弦频率 (Hz)
};

// ============================================================================
// 仿真配置
// ============================================================================

struct SimulationConfig {
    std::string name = "Simulation";
    SignalType duration = 0;       // 仿真时长 (s)
    SignalType startTime = 0;      // 开始时间 (s)

    bool generateClutter = true;
    bool generateTargets = true;
    bool addNoise = true;

    std::string beamCodeFile = ""; // 波位码文件路径
    std::string outputFile = "";   // 输出文件路径
    std::string clutterDataFile = "";// 杂波数据文件路径

    size_t clutterPoolSize = 100;  // 杂波池大小
    size_t clutterSequenceLength = 0; // 杂波序列长度
};

// ============================================================================
// 辅助函数
// ============================================================================

inline SignalType deg2rad(SignalType deg) {
    return deg * PI / 180.0;
}

inline SignalType rad2deg(SignalType rad) {
    return rad * 180.0 / PI;
}

inline SignalType dB2linear(SignalType dB) {
    return std::pow(10.0, dB / 10.0);
}

inline SignalType linear2dB(SignalType linear) {
    return 10.0 * std::log10(linear);
}
