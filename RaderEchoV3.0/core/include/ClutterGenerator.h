#pragma once

#include "Types.h"
#include <vector>
#include <string>
#include <memory>

/**
 * @brief 零记忆非线性变换 (Zero Memory Non-Linear) 杂波生成器
 * @details 通过非线性变换将高斯分布转换为目标分布
 */
class ZMNLGenerator {
public:
    explicit ZMNLGenerator(ClutterDistributionType type = ClutterDistributionType::Rayleigh);
    ~ZMNLGenerator();

    ComplexVector generate(size_t length, const ClutterParams& params);
    ComplexVector generateCorrelated(const ComplexVector& gaussianSeq, const ClutterParams& params);
    void setDistributionType(ClutterDistributionType type);

private:
    ComplexVector transformRayleigh(const ComplexVector& input, SignalType scale);
    ComplexVector transformWeibull(const ComplexVector& input, SignalType scale, SignalType shape);
    ComplexVector transformLogNormal(const ComplexVector& input, SignalType mean, SignalType stddev);
    ComplexVector transformKDistribution(const ComplexVector& input, SignalType shape, SignalType scale);
    SignalType erf(SignalType x);

private:
    ClutterDistributionType m_type;
};

/**
 * @brief 球不变随机过程 (Spherically Invariant Random Process) 杂波生成器
 * @details SIRP 模型 = 高斯过程 × 非负随机调制
 */
class SIRPGenerator {
public:
    explicit SIRPGenerator(ClutterDistributionType type = ClutterDistributionType::K_Distribution);
    ~SIRPGenerator();

    ComplexVector generate(size_t length, const ClutterParams& params, const SpectrumParams& spectrumParams);
    void setDistributionType(ClutterDistributionType type);
    void updateSpectrumFilter(const SpectrumParams& params);

private:
    ComplexVector generateGaussianProcess(size_t length);
    ComplexVector spectralShaping(const ComplexVector& input, const SpectrumParams& params);
    std::vector<SignalType> generateModulation(size_t length, ClutterDistributionType type, const ClutterParams& params);

private:
    ClutterDistributionType m_type;
    std::vector<SignalType> m_filterCoeffs;
    size_t m_filterLength = 256;
};

/**
 * @brief 杂波生成器工厂
 */
class ClutterGeneratorFactory {
public:
    static std::unique_ptr<ZMNLGenerator> createZMNL(ClutterDistributionType type);
    static std::unique_ptr<SIRPGenerator> createSIRP(ClutterDistributionType type);
};
