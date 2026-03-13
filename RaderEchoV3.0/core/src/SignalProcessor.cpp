#include "SignalProcessor.h"
#include "Logger.h"
#include <cmath>
#include <random>
#include <algorithm>
#include <numeric>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================================
// Signal Processor Implementation
// ============================================================================

SignalProcessor::SignalProcessor() = default;

SignalProcessor::~SignalProcessor() = default;

void SignalProcessor::initialize(const RadarSystemParams& radarParams,
                                  const AntennaParams& antennaParams) {
    m_radarParams = radarParams;
    m_antennaParams = antennaParams;

    // 生成参考信号（用于脉压）
    m_referenceSignal = generateLFMWaveform(radarParams.pulseWidth,
                                             radarParams.bandwidth,
                                             radarParams.sampleRate);

    m_initialized = true;
    LOG_INFO("Signal processor initialized");
}

SignalType SignalProcessor::calculateAntennaGain(SignalType azAngle, SignalType elAngle,
                                                  SignalType azSteer, SignalType elSteer) {
    SignalType azGain = calculateAzimuthGain(azAngle, azSteer);
    SignalType elGain = calculateElevationGain(elAngle, elSteer);

    // 双程增益（发射×接收）
    SignalType oneWayGain = azGain * elGain;
    return oneWayGain * oneWayGain;
}

SignalType SignalProcessor::calculateAzimuthGain(SignalType azAngle, SignalType azSteer) {
    SignalType k = 2 * M_PI / m_antennaParams.wavelength;
    SignalType d = m_antennaParams.elementSpacing;
    int N = m_antennaParams.numElementsAz;

    SignalType sinAz = std::sin(azAngle) - std::sin(azSteer);
    SignalType psi = k * d * sinAz;

    if (std::abs(psi) < 1e-10) {
        return 1.0;
    }

    SignalType gain = std::abs(std::sin(N * psi / 2) / (N * std::sin(psi / 2)));
    return gain * gain;  // 功率增益
}

SignalType SignalProcessor::calculateElevationGain(SignalType elAngle, SignalType elSteer) {
    SignalType k = 2 * M_PI / m_antennaParams.wavelength;
    SignalType d = m_antennaParams.elementSpacing;
    int N = m_antennaParams.numElementsEl;

    SignalType sinEl = std::sin(elAngle) - std::sin(elSteer);
    SignalType psi = k * d * sinEl;

    if (std::abs(psi) < 1e-10) {
        return 1.0;
    }

    SignalType gain = std::abs(std::sin(N * psi / 2) / (N * std::sin(psi / 2)));
    return gain * gain;  // 功率增益
}

SignalType SignalProcessor::calculateTargetPower(SignalType range, SignalType rcs,
                                                  SignalType gain) {
    // 雷达方程：Pr = (Pt * G^2 * lambda^2 * sigma) / ((4*PI)^3 * R^4)
    SignalType Pt = m_radarParams.peakPower;
    SignalType lambda = m_radarParams.wavelength;
    SignalType L = std::pow(10.0, m_radarParams.loss / 10.0);

    SignalType Pr = (Pt * gain * gain * lambda * lambda * rcs) /
                    (std::pow(4 * M_PI, 3) * std::pow(range, 4) * L);

    return Pr;
}

ComplexVector SignalProcessor::generateTargetEcho(const TargetParams& target,
                                                   int pulseIndex,
                                                   SignalType currentTime) {
    ComplexVector echo(m_radarParams.numRangeBins, Complex(0, 0));

    // 计算目标当前距离
    SignalType range = target.position.range();
    if (range < m_radarParams.minRange || range > m_radarParams.maxRange) {
        return echo;
    }

    // 计算目标所在的距离门
    int rangeBin = static_cast<int>((range - m_radarParams.minRange) / m_radarParams.rangeBinSize);
    if (rangeBin < 0 || rangeBin >= m_radarParams.numRangeBins) {
        return echo;
    }

    // 计算回波功率
    SignalType azAngle = target.position.azimuth();
    SignalType elAngle = target.position.elevation();

    // 获取当前波束指向（这里假设指向 0，实际应从模拟器传入）
    SignalType azSteer = 0;
    SignalType elSteer = 0;

    SignalType gain = calculateAntennaGain(azAngle, elAngle, azSteer, elSteer);
    SignalType power = calculateTargetPower(range, target.currentRCS, gain);

    // 计算时延和相位
    SignalType delay = 2 * range / C_LIGHT;
    SignalType phase = -2 * M_PI * m_radarParams.frequency * delay;

    // 计算多普勒频移
    SignalType radialVel = target.velocity.radialVelocity(target.position);
    SignalType dopplerFreq = 2 * radialVel / m_radarParams.wavelength;
    SignalType dopplerPhase = 2 * M_PI * dopplerFreq * pulseIndex / m_radarParams.prf;

    // 生成目标信号
    SignalType amplitude = std::sqrt(power);
    SignalType totalPhase = phase + dopplerPhase;

    // 在对应的距离门注入信号
    if (rangeBin >= 0 && rangeBin < m_radarParams.numRangeBins) {
        echo[rangeBin] = Complex(amplitude * std::cos(totalPhase),
                                 amplitude * std::sin(totalPhase));
    }

    return echo;
}

ComplexVector SignalProcessor::generateLFMWaveform(SignalType pulseWidth,
                                                    SignalType bandwidth,
                                                    SignalType sampleRate) {
    int numSamples = static_cast<int>(pulseWidth * sampleRate);
    ComplexVector signal(numSamples);

    SignalType chirpRate = bandwidth / pulseWidth;

    for (int n = 0; n < numSamples; ++n) {
        SignalType t = n / sampleRate;
        SignalType phase = M_PI * chirpRate * t * t;
        signal[n] = Complex(std::cos(phase), std::sin(phase));
    }

    return signal;
}

ComplexVector SignalProcessor::matchedFilter(const ComplexVector& echo,
                                              const ComplexVector& reference) {
    size_t echoLen = echo.size();
    size_t refLen = reference.size();
    size_t outputLen = echoLen + refLen - 1;

    ComplexVector output(outputLen, Complex(0, 0));

    // 生成参考信号的共轭反转
    ComplexVector matchedRef(refLen);
    for (size_t i = 0; i < refLen; ++i) {
        matchedRef[i] = std::conj(reference[refLen - 1 - i]);
    }

    // 卷积（匹配滤波）
    for (size_t n = 0; n < outputLen; ++n) {
        for (size_t k = 0; k < refLen; ++k) {
            if (n - k < echoLen) {
                output[n] += echo[n - k] * matchedRef[k];
            }
        }
    }

    return output;
}

SignalType SignalProcessor::calculateSigma0(SignalType grazingAngle, SignalType windSpeed,
                                             SignalType frequency) {
    // 简化的海杂波模型（X 波段）
    // sigma0 = 0.001 * (grazing_angle)^1.5 * (wind_speed)^2

    SignalType K = 0.001;
    SignalType gamma = 1.5;
    SignalType delta = 2.0;

    return K * std::pow(grazingAngle, gamma) * std::pow(windSpeed + 1.0, delta);
}

SignalType SignalProcessor::calculateClutterPower(const ClutterCell& cell,
                                                   const ClutterParams& clutterParams) {
    // 杂波功率计算同 GridManager 中的实现
    SignalType Pt = m_radarParams.peakPower;
    SignalType G = cell.antennaGain;
    SignalType lambda = m_radarParams.wavelength;
    SignalType R = cell.range;
    SignalType L = std::pow(10.0, m_radarParams.loss / 10.0);

    SignalType sigma0 = clutterParams.cellParams.sigma0;
    SignalType A_cell = cell.rangeSpan * (cell.range * cell.azimuthSpan);

    SignalType Pc = (Pt * G * G * lambda * lambda * sigma0 * A_cell) /
                    (std::pow(4 * M_PI, 3) * std::pow(R, 4) * L);

    return Pc;
}

ComplexVector SignalProcessor::generateThermalNoise(size_t length, SignalType noisePower) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<SignalType> dist(0, 1);

    SignalType sigma = std::sqrt(noisePower / 2.0);

    ComplexVector noise(length);
    for (size_t i = 0; i < length; ++i) {
        SignalType real = dist(gen) * sigma;
        SignalType imag = dist(gen) * sigma;
        noise[i] = Complex(real, imag);
    }

    return noise;
}

SignalType SignalProcessor::calculateNoisePower(SignalType bandwidth, SignalType noiseFigure,
                                                 SignalType temperature) {
    // 热噪声功率：N = k * T * B * F
    SignalType k = 1.38e-23;  // Boltzmann 常数
    SignalType F = std::pow(10.0, noiseFigure / 10.0);  // 噪声系数转线性

    return k * temperature * bandwidth * F;
}

void SignalProcessor::cart2sph(SignalType x, SignalType y, SignalType z,
                                SignalType& range, SignalType& azimuth, SignalType& elevation) {
    range = std::sqrt(x * x + y * y + z * z);
    azimuth = std::atan2(y, x);
    elevation = std::atan2(z, std::sqrt(x * x + y * y));
}

void SignalProcessor::sph2cart(SignalType range, SignalType azimuth, SignalType elevation,
                                SignalType& x, SignalType& y, SignalType& z) {
    SignalType r_xy = range * std::cos(elevation);
    x = r_xy * std::cos(azimuth);
    y = r_xy * std::sin(azimuth);
    z = range * std::sin(elevation);
}
