#include "ClutterGenerator.h"
#include <random>
#include <cmath>
#include <algorithm>
#include <numeric>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================================
// ZMNL Generator Implementation
// ============================================================================

ZMNLGenerator::ZMNLGenerator(ClutterDistributionType type)
    : m_type(type) {}

ZMNLGenerator::~ZMNLGenerator() = default;

void ZMNLGenerator::setDistributionType(ClutterDistributionType type) {
    m_type = type;
}

ComplexVector ZMNLGenerator::generate(size_t length, const ClutterParams& params) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<SignalType> dist(0.0, 1.0);

    ComplexVector gaussianSeq(length);
    for (size_t i = 0; i < length; ++i) {
        SignalType real = dist(gen);
        SignalType imag = dist(gen);
        gaussianSeq[i] = Complex(real, imag);
    }

    return generateCorrelated(gaussianSeq, params);
}

ComplexVector ZMNLGenerator::generateCorrelated(const ComplexVector& gaussianSeq,
                                                  const ClutterParams& params) {
    switch (m_type) {
        case ClutterDistributionType::Rayleigh:
            return transformRayleigh(gaussianSeq, params.weibullScale);
        case ClutterDistributionType::Weibull:
            return transformWeibull(gaussianSeq, params.weibullScale, params.weibullShape);
        case ClutterDistributionType::LogNormal:
            return transformLogNormal(gaussianSeq, params.logNormalMean, params.logNormalStd);
        case ClutterDistributionType::K_Distribution:
            return transformKDistribution(gaussianSeq, params.kDistShape, params.weibullScale);
        default:
            return transformRayleigh(gaussianSeq, 1.0);
    }
}

ComplexVector ZMNLGenerator::transformRayleigh(const ComplexVector& input, SignalType scale) {
    ComplexVector output(input.size());
    SignalType sigma = scale / std::sqrt(2.0);

    for (size_t i = 0; i < input.size(); ++i) {
        SignalType r = std::abs(input[i]);
        SignalType phase = std::arg(input[i]);
        SignalType rayleighAmp = sigma * r;
        output[i] = Complex(rayleighAmp * std::cos(phase), rayleighAmp * std::sin(phase));
    }
    return output;
}

ComplexVector ZMNLGenerator::transformWeibull(const ComplexVector& input,
                                                SignalType scale,
                                                SignalType shape) {
    ComplexVector output(input.size());
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<SignalType> uniform(0.0, 1.0);

    for (size_t i = 0; i < input.size(); ++i) {
        SignalType phase = std::arg(input[i]);
        SignalType u = uniform(gen);
        SignalType weibullAmp = scale * std::pow(-std::log(1.0 - u), 1.0 / shape);
        output[i] = Complex(weibullAmp * std::cos(phase), weibullAmp * std::sin(phase));
    }
    return output;
}

ComplexVector ZMNLGenerator::transformLogNormal(const ComplexVector& input,
                                                  SignalType mean,
                                                  SignalType stddev) {
    ComplexVector output(input.size());
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<SignalType> normalDist(mean, stddev);

    for (size_t i = 0; i < input.size(); ++i) {
        SignalType phase = std::arg(input[i]);
        SignalType logNormalAmp = std::exp(normalDist(gen));
        output[i] = Complex(logNormalAmp * std::cos(phase), logNormalAmp * std::sin(phase));
    }
    return output;
}

ComplexVector ZMNLGenerator::transformKDistribution(const ComplexVector& input,
                                                      SignalType shape,
                                                      SignalType scale) {
    ComplexVector output(input.size());
    std::random_device rd;
    std::mt19937 gen(rd());

    std::gamma_distribution<SignalType> gammaDist(shape, 1.0);

    for (size_t i = 0; i < input.size(); ++i) {
        SignalType r = std::abs(input[i]);
        SignalType phase = std::arg(input[i]);
        SignalType gamma = gammaDist(gen);
        SignalType kAmp = scale * r * std::sqrt(gamma / shape);
        output[i] = Complex(kAmp * std::cos(phase), kAmp * std::sin(phase));
    }
    return output;
}

SignalType ZMNLGenerator::erf(SignalType x) {
    SignalType sign = (x >= 0) ? 1.0 : -1.0;
    x = std::abs(x);

    SignalType a1 = 0.254829592;
    SignalType a2 = -0.284496736;
    SignalType a3 = 1.421413741;
    SignalType a4 = -1.453152027;
    SignalType a5 = 1.061405429;
    SignalType p = 0.3275911;

    SignalType t = 1.0 / (1.0 + p * x);
    SignalType y = 1.0 - (((((a5 * t + a4) * t) + a3) * t + a2) * t + a1) * t * std::exp(-x * x);

    return sign * y;
}

// ============================================================================
// SIRP Generator Implementation
// ============================================================================

SIRPGenerator::SIRPGenerator(ClutterDistributionType type)
    : m_type(type) {}

SIRPGenerator::~SIRPGenerator() = default;

void SIRPGenerator::setDistributionType(ClutterDistributionType type) {
    m_type = type;
}

void SIRPGenerator::updateSpectrumFilter(const SpectrumParams& params) {
    m_filterCoeffs.resize(m_filterLength);

    SignalType sigma = params.bandwidth / (2.0 * std::sqrt(2.0 * std::log(2.0)));
    SignalType center = params.centerFreq;

    for (size_t i = 0; i < m_filterLength; ++i) {
        SignalType f = static_cast<SignalType>(i) / m_filterLength;
        f = (f < 0.5) ? f : f - 1.0;
        SignalType df = f - center;
        m_filterCoeffs[i] = std::exp(-0.5 * df * df / (sigma * sigma));
    }
}

ComplexVector SIRPGenerator::generate(size_t length,
                                       const ClutterParams& params,
                                       const SpectrumParams& spectrumParams) {
    ComplexVector gaussianProc = generateGaussianProcess(length);
    ComplexVector shapedProc = spectralShaping(gaussianProc, spectrumParams);

    auto modulation = generateModulation(length, m_type, params);

    ComplexVector output(length);
    for (size_t i = 0; i < length; ++i) {
        output[i] = shapedProc[i] * std::sqrt(modulation[i]);
    }

    return output;
}

ComplexVector SIRPGenerator::generateGaussianProcess(size_t length) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<SignalType> dist(0.0, 1.0);

    ComplexVector output(length);
    for (size_t i = 0; i < length; ++i) {
        SignalType real = dist(gen);
        SignalType imag = dist(gen);
        output[i] = Complex(real, imag);
    }
    return output;
}

ComplexVector SIRPGenerator::spectralShaping(const ComplexVector& input,
                                              const SpectrumParams& params) {
    size_t N = input.size();

    SignalType sigma = params.bandwidth;
    SignalType center = params.centerFreq;

    ComplexVector output(N);
    for (size_t i = 0; i < N; ++i) {
        SignalType t = static_cast<SignalType>(i);
        SignalType gaussWindow = std::exp(-0.5 * t * t / (N * N * 0.1));
        output[i] = input[i] * gaussWindow;
    }

    return output;
}

std::vector<SignalType> SIRPGenerator::generateModulation(size_t length,
                                                           ClutterDistributionType type,
                                                           const ClutterParams& params) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::vector<SignalType> modulation(length);

    switch (type) {
        case ClutterDistributionType::K_Distribution: {
            std::gamma_distribution<SignalType> gammaDist(params.kDistShape, 1.0);
            for (size_t i = 0; i < length; ++i) {
                modulation[i] = gammaDist(gen);
            }
            break;
        }
        case ClutterDistributionType::LogNormal: {
            std::lognormal_distribution<SignalType> lognormDist(params.logNormalMean,
                                                                 params.logNormalStd);
            for (size_t i = 0; i < length; ++i) {
                modulation[i] = lognormDist(gen);
            }
            break;
        }
        default: {
            std::fill(modulation.begin(), modulation.end(), 1.0);
        }
    }

    return modulation;
}

// ============================================================================
// Clutter Generator Factory
// ============================================================================

std::unique_ptr<ZMNLGenerator> ClutterGeneratorFactory::createZMNL(ClutterDistributionType type) {
    return std::make_unique<ZMNLGenerator>(type);
}

std::unique_ptr<SIRPGenerator> ClutterGeneratorFactory::createSIRP(ClutterDistributionType type) {
    return std::make_unique<SIRPGenerator>(type);
}
