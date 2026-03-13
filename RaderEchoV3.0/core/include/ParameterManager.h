#pragma once

#include "Types.h"
#include "BeamCodeManager.h"
#include <vector>
#include <string>
#include <fstream>

/**
 * @brief 参数管理器
 * @details 负责从文件加载雷达、天线、杂波等参数
 */
class ParameterManager {
public:
    ParameterManager();
    ~ParameterManager();

    SimulationConfig loadFromFile(const std::string& filename);
    bool saveToFile(const std::string& filename,
                   const SimulationConfig& config,
                   const RadarSystemParams& radarParams,
                   const AntennaParams& antennaParams,
                   const ClutterParams& clutterParams);

    SimulationConfig getDefaultConfig() const;
    RadarSystemParams getDefaultRadarParams() const;
    AntennaParams getDefaultAntennaParams() const;
    ClutterParams getDefaultClutterParams() const;

    bool validateParams(const SimulationConfig& config,
                       const RadarSystemParams& radarParams,
                       const AntennaParams& antennaParams);

private:
    void parseLine(const std::string& line, SimulationConfig& config,
                   RadarSystemParams& radarParams, AntennaParams& antennaParams,
                   ClutterParams& clutterParams);
    void updateDerivedParams(RadarSystemParams& radarParams,
                            AntennaParams& antennaParams);
};
