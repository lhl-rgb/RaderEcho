#pragma once

#include "Types.h"
#include <vector>
#include <string>
#include <memory>

/**
 * @brief 零记忆非线性变换 (Zero Memory Non-Linear) 杂波生成器
 * @details 通过非线性变换将高斯分布转换为目标分布
 *
 * 工作原理：
 * 1. 生成独立高斯随机序列
 * 2. 通过累积分布函数匹配进行非线性变换
 * 3. 输出指定分布的杂波序列
 */
class ZMNLGenerator {
public:
    explicit ZMNLGenerator(ClutterDistributionType type = ClutterDistributionType::Rayleigh);
    ~ZMNLGenerator();

    /**
     * @brief 生成指定长度的杂波序列
     * @param length 序列长度
     * @param params 杂波参数
     * @return 复数杂波序列
     */
    ComplexVector generate(size_t length, const ClutterParams& params);

    /**
     * @brief 生成相关杂波序列（与输入高斯序列相关）
     * @param gaussianSeq 输入高斯序列
     * @param params 杂波参数
     * @return 复数杂波序列
     */
    ComplexVector generateCorrelated(const ComplexVector& gaussianSeq, const ClutterParams& params);

    /**
     * @brief 设置分布类型
     */
    void setDistributionType(ClutterDistributionType type);

private:
    /**
     * @brief Rayleigh 分布变换
     */
    ComplexVector transformRayleigh(const ComplexVector& input, SignalType scale);

    /**
     * @brief Weibull 分布变换
     */
    ComplexVector transformWeibull(const ComplexVector& input, SignalType scale, SignalType shape);

    /**
     * @brief LogNormal 分布变换
     */
    ComplexVector transformLogNormal(const ComplexVector& input, SignalType mean, SignalType stddev);

    /**
     * @brief K 分布变换
     * @details K 分布 = Rayleigh * sqrt(Gamma), Gamma 为调制分量
     */
    ComplexVector transformKDistribution(const ComplexVector& input, SignalType shape, SignalType scale);

    /**
     * @brief 误差函数（用于正态分布 CDF）
     */
    SignalType erf(SignalType x);

private:
    ClutterDistributionType m_type;
};

/**
 * @brief 球不变随机过程 (Spherically Invariant Random Process) 杂波生成器
 * @details SIRP 模型 = 高斯过程 × 非负随机调制
 *
 * 工作原理：
 * 1. 生成高斯随机过程
 * 2. 通过频谱整形滤波器获得时间相关性
 * 3. 乘以随机调制分量（服从特定分布）
 */
class SIRPGenerator {
public:
    explicit SIRPGenerator(ClutterDistributionType type = ClutterDistributionType::K_Distribution);
    ~SIRPGenerator();

    /**
     * @brief 生成指定长度的杂波序列
     * @param length 序列长度
     * @param params 杂波参数
     * @param spectrumParams 频谱参数
     * @return 复数杂波序列
     */
    ComplexVector generate(size_t length, const ClutterParams& params, const SpectrumParams& spectrumParams);

    /**
     * @brief 设置分布类型
     */
    void setDistributionType(ClutterDistributionType type);

    /**
     * @brief 更新频谱滤波器系数
     */
    void updateSpectrumFilter(const SpectrumParams& params);

private:
    /**
     * @brief 生成高斯随机过程
     */
    ComplexVector generateGaussianProcess(size_t length);

    /**
     * @brief 频谱整形滤波
     * @details 使用 FFT 方法实现频谱整形
     */
    ComplexVector spectralShaping(const ComplexVector& input, const SpectrumParams& params);

    /**
     * @brief 生成调制分量（K 分布用逆 Gamma 分布）
     */
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
