#pragma once

#include "Types.h"
#include "TargetManager.h"
#include "ClutterManager.h"
#include "GridManager.h"
#include "SignalProcessor.h"
#include "ParameterManager.h"
#include "BeamCodeManager.h"
#include <vector>
#include <string>
#include <memory>

/**
 * @brief 扫描结果数据结构
 */
struct ScanData {
    std::vector<ComplexVector> pulseData;  // 每个脉冲的回波数据
    int scanIndex = 0;                      // 扫描索引
    int beamIndex = 0;                      // 波束索引
    SignalType scanTime = 0;                // 扫描时间
    SignalType azimuth = 0;                 // 方位角
    SignalType elevation = 0;               // 俯仰角
};

/**
 * @brief CPI 数据结构
 */
struct CPIData {
    ComplexMatrix echoMatrix;           // 回波矩阵 [range x pulses]
    int cpiIndex = 0;                   // CPI 索引
    int numPulses = 0;                  // 脉冲数
    int numRangeBins = 0;               // 距离门数
    SignalType startTime = 0;           // CPI 开始时间
    BeamPosition beamPos;               // 波束指向
};

/**
 * @brief 相控阵雷达回波模拟器
 * @details 主模拟器类，整合所有模块完成回波仿真
 *
 * 工作流程：
 * 1. 加载参数和波位码
 * 2. 初始化杂波内存池
 * 3. 对每个波束指向：
 *    - 生成网格划分
 *    - 计算每个网格的杂波功率
 *    - 从内存池获取杂波序列并调制
 *    - 生成目标回波（如有）
 *    - 累加所有网格贡献得到每个距离门的回波
 * 4. 输出回波数据
 */
class PhasedArrayRadarSimulator {
public:
    PhasedArrayRadarSimulator();
    ~PhasedArrayRadarSimulator();

    // ========================================================================
    // 初始化和配置
    // ========================================================================

    /**
     * @brief 初始化模拟器
     * @param configPath 配置文件路径
     * @return 是否成功
     */
    bool initialize(const std::string& configPath);

    /**
     * @brief 手动初始化（不使用配置文件）
     */
    bool initialize(const SimulationConfig& config,
                   const RadarSystemParams& radarParams,
                   const AntennaParams& antennaParams,
                   const ClutterParams& clutterParams);

    /**
     * @brief 重置模拟器状态
     */
    void reset();

    /**
     * @brief 停止仿真
     */
    void stop();

    // ========================================================================
    // 目标管理
    // ========================================================================

    /**
     * @brief 添加目标
     */
    size_t addTarget(const TargetParams& target);

    /**
     * @brief 删除目标
     */
    void removeTarget(size_t id);

    /**
     * @brief 更新目标
     */
    void updateTarget(size_t id, const TargetParams& target);

    /**
     * @brief 获取所有目标
     */
    std::vector<TargetParams> getAllTargets() const;

    // ========================================================================
    // 仿真执行
    // ========================================================================

    /**
     * @brief 执行单次扫描
     * @return 扫描数据
     */
    ScanData runSingleScan();

    /**
     * @brief 执行单个 CPI
     * @param beamIndex 波束索引
     * @return CPI 数据
     */
    CPIData runSingleCPI(int beamIndex);

    /**
     * @brief 执行多次扫描
     * @param numScans 扫描次数
     * @param outputDir 输出目录
     */
    void runMultipleScans(int numScans, const std::string& outputDir = "../out/data");

    /**
     * @brief 执行完整仿真
     */
    void runFullSimulation();

    // ========================================================================
    // 数据访问
    // ========================================================================

    /**
     * @brief 获取模拟器状态
     */
    bool isRunning() const { return m_running; }
    bool isInitialized() const { return m_initialized; }

    /**
     * @brief 获取当前扫描索引
     */
    int getCurrentScanIndex() const { return m_currentScan; }

    /**
     * @brief 获取当前 CPI 索引
     */
    int getCurrentCPIIndex() const { return m_currentCPI; }

    /**
     * @brief 获取配置
     */
    const SimulationConfig& getConfig() const { return m_config; }
    const RadarSystemParams& getRadarParams() const { return m_radarParams; }
    const AntennaParams& getAntennaParams() const { return m_antennaParams; }
    const ClutterParams& getClutterParams() const { return m_clutterParams; }

private:
    // ========================================================================
    // 内部方法
    // ========================================================================

    /**
     * @brief 更新仿真状态
     */
    void updateSimulationState(SignalType dt);

    /**
     * @brief 计算网格对距离门的贡献
     */
    void accumulateCellContribution(const ClutterCell& cell,
                                    const ComplexVector& clutterSeq,
                                    ComplexVector& rangeProfile);

    /**
     * @brief 保存扫描数据
     */
    void saveScanData(const ScanData& data, const std::string& outputDir);

    /**
     * @brief 保存 CPI 数据
     */
    void saveCPIData(const CPIData& data, const std::string& outputDir);

    /**
     * @brief 加载波位码
     */
    bool loadBeamCodes(const std::string& filename);

// ========================================================================
// 成员变量
// ========================================================================

private:
    // 配置参数
    SimulationConfig m_config;
    RadarSystemParams m_radarParams;
    AntennaParams m_antennaParams;
    ClutterParams m_clutterParams;
    SpectrumParams m_spectrumParams;

    // 功能模块
    std::unique_ptr<TargetManager> m_targetManager;
    std::unique_ptr<ClutterManager> m_clutterManager;
    std::unique_ptr<GridManager> m_gridManager;
    std::unique_ptr<SignalProcessor> m_signalProcessor;
    std::unique_ptr<BeamCodeManager> m_beamCodeManager;

    // 仿真状态
    bool m_initialized = false;
    bool m_running = false;
    int m_currentScan = 0;
    int m_currentCPI = 0;
    int m_currentPulse = 0;
    SignalType m_currentTime = 0;

    // 缓存数据
    ComplexVector m_referenceSignal;  // 参考信号
    std::vector<ClutterCell> m_currentGrid;  // 当前网格
};
