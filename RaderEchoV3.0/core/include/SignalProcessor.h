#pragma once

#include "Types.h"
#include <vector>
#include <complex>

/**
 * @brief 信号处理器
 * @details 负责回波信号生成、脉压处理、天线增益计算等
 */
class SignalProcessor {
public:
    SignalProcessor();
    ~SignalProcessor();

    /**
     * @brief 初始化信号处理器
     */
    void initialize(const RadarSystemParams& radarParams,
                   const AntennaParams& antennaParams);

    // ========================================================================
    // 天线增益计算
    // ========================================================================

    /**
     * @brief 计算天线方向图增益
     * @param azAngle 目标方位角 (rad)
     * @param elAngle 目标俯仰角 (rad)
     * @param azSteer 波束方位指向 (rad)
     * @param elSteer 波束俯仰指向 (rad)
     * @return 双程增益（线性值）
     */
    SignalType calculateAntennaGain(SignalType azAngle, SignalType elAngle,
                                    SignalType azSteer, SignalType elSteer);

    /**
     * @brief 计算单程方位增益
     */
    SignalType calculateAzimuthGain(SignalType azAngle, SignalType azSteer);

    /**
     * @brief 计算单程俯仰增益
     */
    SignalType calculateElevationGain(SignalType elAngle, SignalType elSteer);

    // ========================================================================
    // 目标回波计算
    // ========================================================================

    /**
     * @brief 计算目标回波功率
     * @param range 目标距离 (m)
     * @param rcs 目标 RCS (m^2)
     * @param gain 天线增益（线性值）
     * @return 接收功率 (W)
     *
     * 雷达方程：
     * Pr = (Pt * G^2 * lambda^2 * sigma) / ((4*PI)^3 * R^4)
     */
    SignalType calculateTargetPower(SignalType range, SignalType rcs, SignalType gain);

    /**
     * @brief 生成目标回波信号
     * @param target 目标参数
     * @param pulseIndex 脉冲索引
     * @param currentTime 当前时间
     * @return 复数回波信号
     */
    ComplexVector generateTargetEcho(const TargetParams& target,
                                      int pulseIndex,
                                      SignalType currentTime);

    /**
     * @brief 生成 LFM 发射信号
     * @param pulseWidth 脉宽
     * @param bandwidth 带宽
     * @param sampleRate 采样率
     * @return 复数发射信号
     */
    ComplexVector generateLFMWaveform(SignalType pulseWidth,
                                       SignalType bandwidth,
                                       SignalType sampleRate);

    /**
     * @brief 匹配滤波（脉冲压缩）
     * @param echo 回波信号
     * @param reference 参考信号
     * @return 脉压后信号
     */
    ComplexVector matchedFilter(const ComplexVector& echo,
                                const ComplexVector& reference);

    // ========================================================================
    // 杂波处理
    // ========================================================================

    /**
     * @brief 计算杂波单元的后向散射系数
     * @param grazingAngle 掠射角 (rad)
     * @param windSpeed 风速 (m/s)
     * @param frequency 雷达频率 (Hz)
     * @return sigma0
     */
    SignalType calculateSigma0(SignalType grazingAngle, SignalType windSpeed,
                               SignalType frequency);

    /**
     * @brief 计算杂波功率
     * @param cell 杂波单元
     * @param clutterParams 杂波参数
     * @return 杂波功率
     */
    SignalType calculateClutterPower(const ClutterCell& cell,
                                     const ClutterParams& clutterParams);

    // ========================================================================
    // 噪声生成
    // ========================================================================

    /**
     * @brief 生成热噪声
     * @param length 样本长度
     * @param noisePower 噪声功率
     * @return 复数噪声序列
     */
    ComplexVector generateThermalNoise(size_t length, SignalType noisePower);

    /**
     * @brief 计算系统噪声功率
     * @param bandwidth 带宽
     * @param noiseFigure 噪声系数 (dB)
     * @param temperature 温度 (K)
     * @return 噪声功率 (W)
     */
    SignalType calculateNoisePower(SignalType bandwidth, SignalType noiseFigure,
                                   SignalType temperature = 290.0);

    // ========================================================================
    // 坐标转换
    // ========================================================================

    /**
     * @brief 笛卡尔坐标转球坐标
     */
    static void cart2sph(SignalType x, SignalType y, SignalType z,
                         SignalType& range, SignalType& azimuth, SignalType& elevation);

    /**
     * @brief 球坐标转笛卡尔坐标
     */
    static void sph2cart(SignalType range, SignalType azimuth, SignalType elevation,
                         SignalType& x, SignalType& y, SignalType& z);

private:
    RadarSystemParams m_radarParams;
    AntennaParams m_antennaParams;

    ComplexVector m_referenceSignal;  // 参考信号（用于脉压）
    bool m_initialized = false;
};
