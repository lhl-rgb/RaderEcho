#pragma once

#include "Types.h"
#include <vector>
#include <string>
#include <memory>

/**
 * @brief 网格管理器
 * @details 负责照射区域的网格划分、网格参数计算、杂波功率计算
 *
 * 网格划分策略：
 * - 距离向：按距离分辨率划分
 * - 方位向：按波束宽度划分
 * - 俯仰向：按波束宽度划分
 */
class GridManager {
public:
    explicit GridManager();
    ~GridManager();

    /**
     * @brief 初始化网格系统
     * @param radarParams 雷达参数
     * @param antennaParams 天线参数
     */
    void initialize(const RadarSystemParams& radarParams, const AntennaParams& antennaParams);

    /**
     * @brief 为特定波束指向生成网格
     * @param beamPos 波束指向
     * @param numRangeBins 距离门数量
     * @param numAzBins 方位划分数量
     * @param numElBins 俯仰划分数量
     * @return 网格单元列表
     */
    std::vector<ClutterCell> generateGridForBeam(const BeamPosition& beamPos,
                                                  int numRangeBins,
                                                  int numAzBins = 1,
                                                  int numElBins = 1);

    /**
     * @brief 计算网格单元的天线增益
     * @param cell 网格单元
     * @param beamPos 波束指向
     * @return 天线增益（线性值）
     */
    SignalType calculateCellGain(const ClutterCell& cell, const BeamPosition& beamPos);

    /**
     * @brief 计算网格单元的杂波功率
     * @param cell 网格单元
     * @param clutterParams 杂波参数
     * @param radarParams 雷达参数
     * @return 杂波功率
     *
     * 计算公式：
     * Pc = (Pt * G^2 * lambda^2 * sigma0 * A_cell) / ((4*PI)^3 * R^4 * L)
     */
    SignalType calculateCellClutterPower(const ClutterCell& cell,
                                          const ClutterParams& clutterParams,
                                          const RadarSystemParams& radarParams);

    /**
     * @brief 计算网格单元的有效照射面积
     * @param cell 网格单元
     * @return 有效面积 (m^2)
     */
    SignalType calculateEffectiveArea(const ClutterCell& cell);

    /**
     * @brief 计算掠射角
     * @param range 斜距
     * @param antennaHeight 天线高度
     * @return 掠射角 (rad)
     */
    SignalType calculateGrazingAngle(SignalType range, SignalType antennaHeight);

    /**
     * @brief 获取网格总数
     */
    size_t getTotalCells() const { return m_totalCells; }

    /**
     * @brief 清除网格缓存
     */
    void clear();

private:
    RadarSystemParams m_radarParams;
    AntennaParams m_antennaParams;

    size_t m_totalCells = 0;
    std::vector<std::vector<ClutterCell>> m_gridCache;  // 按波束索引缓存
};
