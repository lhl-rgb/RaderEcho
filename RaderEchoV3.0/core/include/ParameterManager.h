#pragma once

#include "Types.h"
#include <vector>
#include <string>
#include <fstream>

/**
 * @brief 波位码管理器
 * @details 负责波位码文件的读取、解析和波束序列管理
 *
 * 波位码文件格式：
 * # 波位码文件
 * # beam_code azimuth(deg) elevation(deg)
 * 0  0.0  0.0
 * 1  1.5  0.0
 * 2  3.0  0.0
 * ...
 */
class BeamCodeManager {
public:
    BeamCodeManager();
    ~BeamCodeManager();

    /**
     * @brief 从文件加载波位码
     * @param filename 文件路径
     * @return 是否成功
     */
    bool loadFromFile(const std::string& filename);

    /**
     * @brief 从字符串加载波位码
     * @param content 文件内容
     * @return 是否成功
     */
    bool loadFromString(const std::string& content);

    /**
     * @brief 获取指定索引的波束位置
     * @param index 波束索引
     * @return 波束位置（不存在则返回空）
     */
    BeamPosition getBeamAt(size_t index) const;

    /**
     * @brief 获取所有波束位置
     * @return 波束位置列表
     */
    const std::vector<BeamPosition>& getAllBeams() const { return m_beams; }

    /**
     * @brief 获取波束数量
     */
    size_t getBeamCount() const { return m_beams.size(); }

    /**
     * @brief 获取当前波束索引
     */
    size_t getCurrentBeamIndex() const { return m_currentBeamIndex; }

    /**
     * @brief 前进到下一个波束
     * @return 下一个波束位置
     */
    BeamPosition nextBeam();

    /**
     * @brief 重置波束索引
     */
    void reset() { m_currentBeamIndex = 0; }

    /**
     * @brief 设置当前波束索引
     */
    void setCurrentBeamIndex(size_t index) { m_currentBeamIndex = index; }

    /**
     * @brief 清除所有波位码
     */
    void clear();

private:
    std::vector<BeamPosition> m_beams;
    size_t m_currentBeamIndex = 0;
    bool m_loaded = false;
};

/**
 * @brief 参数管理器
 * @details 负责从文件加载雷达、天线、杂波等参数
 *
 * 参数文件格式：
 * # Radar Parameters
 * frequency 9.4e9
 * bandwidth 20e6
 * prf 1200
 * ...
 */
class ParameterManager {
public:
    ParameterManager();
    ~ParameterManager();

    /**
     * @brief 从文件加载所有参数
     * @param filename 文件路径
     * @return 仿真配置
     */
    SimulationConfig loadFromFile(const std::string& filename);

    /**
     * @brief 保存参数到文件
     * @param filename 文件路径
     * @param config 仿真配置
     * @param radarParams 雷达参数
     * @param antennaParams 天线参数
     * @param clutterParams 杂波参数
     * @return 是否成功
     */
    bool saveToFile(const std::string& filename,
                   const SimulationConfig& config,
                   const RadarSystemParams& radarParams,
                   const AntennaParams& antennaParams,
                   const ClutterParams& clutterParams);

    /**
     * @brief 获取默认仿真配置
     */
    SimulationConfig getDefaultConfig() const;

    /**
     * @brief 获取默认雷达参数
     */
    RadarSystemParams getDefaultRadarParams() const;

    /**
     * @brief 获取默认天线参数
     */
    AntennaParams getDefaultAntennaParams() const;

    /**
     * @brief 获取默认杂波参数
     */
    ClutterParams getDefaultClutterParams() const;

    /**
     * @brief 验证参数合法性
     * @param config 仿真配置
     * @param radarParams 雷达参数
     * @param antennaParams 天线参数
     * @return 是否合法
     */
    bool validateParams(const SimulationConfig& config,
                       const RadarSystemParams& radarParams,
                       const AntennaParams& antennaParams);

private:
    /**
     * @brief 解析一行参数
     */
    void parseLine(const std::string& line, SimulationConfig& config,
                   RadarSystemParams& radarParams, AntennaParams& antennaParams,
                   ClutterParams& clutterParams);

    /**
     * @brief 更新推导参数
     */
    void updateDerivedParams(RadarSystemParams& radarParams,
                            AntennaParams& antennaParams);
};
