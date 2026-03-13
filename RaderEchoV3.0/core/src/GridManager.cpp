#include "GridManager.h"
#include "Logger.h"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================================
// Grid Manager Implementation
// ============================================================================

GridManager::GridManager() = default;

GridManager::~GridManager() = default;

void GridManager::initialize(const RadarSystemParams& radarParams,
                              const AntennaParams& antennaParams) {
    m_radarParams = radarParams;
    m_antennaParams = antennaParams;
    m_totalCells = 0;

    LOG_INFO("Grid manager initialized");
    LOG_INFO_SIMPLE("Range bins: " + std::to_string(radarParams.numRangeBins));
}

std::vector<ClutterCell> GridManager::generateGridForBeam(const BeamPosition& beamPos,
                                                           int numRangeBins,
                                                           int numAzBins,
                                                           int numElBins) {
    std::vector<ClutterCell> cells;
    cells.reserve(numRangeBins * numAzBins * numElBins);

    SignalType azBeamwidth_rad = m_antennaParams.azBeamwidth_rad;
    SignalType elBeamwidth_rad = m_antennaParams.elBeamwidth_rad;

    // 计算每个划分的大小
    SignalType azSpan = azBeamwidth_rad / numAzBins;
    SignalType elSpan = elBeamwidth_rad / numElBins;
    SignalType rangeSpan = m_radarParams.rangeBinSize;

    // 生成网格
    for (int r = 0; r < numRangeBins; ++r) {
        for (int az = 0; az < numAzBins; ++az) {
            for (int el = 0; el < numElBins; ++el) {
                ClutterCell cell;

                // 距离
                cell.rangeIndex = r;
                cell.range = m_radarParams.minRange + r * rangeSpan + rangeSpan / 2.0;
                cell.rangeSpan = rangeSpan;

                // 方位（相对于波束中心）
                cell.azimuthIndex = az;
                cell.azimuth = beamPos.azimuth * M_PI / 180.0 +
                               (az - numAzBins / 2.0) * azSpan;
                cell.azimuthSpan = azSpan;

                // 俯仰（相对于波束中心）
                cell.elevationIndex = el;
                cell.elevation = beamPos.elevation * M_PI / 180.0 +
                                 (el - numElBins / 2.0) * elSpan;
                cell.elevationSpan = elSpan;

                // 计算天线增益
                cell.antennaGain = calculateCellGain(cell, beamPos);

                // 计算掠射角
                cell.clutterPower = cell.range;  // 临时存储，实际功率后续计算

                cells.push_back(cell);
            }
        }
    }

    m_totalCells += cells.size();
    m_gridCache.push_back(cells);

    LOG_INFO_SIMPLE("Generated " + std::to_string(cells.size()) + " cells for beam " +
                   std::to_string(beamPos.beamCode));
    return cells;
}

SignalType GridManager::calculateCellGain(const ClutterCell& cell,
                                           const BeamPosition& beamPos) {
    SignalType azSteer = beamPos.azimuth * M_PI / 180.0;
    SignalType elSteer = beamPos.elevation * M_PI / 180.0;

    SignalType azDelta = cell.azimuth - azSteer;
    SignalType elDelta = cell.elevation - elSteer;

    // 使用相控阵方向图公式
    SignalType gain = m_antennaParams.calculateArrayGain(
        cell.azimuth, cell.elevation, azSteer, elSteer);

    // 应用最大增益
    gain *= m_antennaParams.gain_linear;

    return gain;
}

SignalType GridManager::calculateCellClutterPower(const ClutterCell& cell,
                                                   const ClutterParams& clutterParams,
                                                   const RadarSystemParams& radarParams) {
    // 雷达杂波方程：
    // Pc = (Pt * G^2 * lambda^2 * sigma0 * A_cell) / ((4*PI)^3 * R^4 * L)

    SignalType Pt = radarParams.peakPower;
    SignalType G = cell.antennaGain;
    SignalType lambda = radarParams.wavelength;
    SignalType R = cell.range;
    SignalType L = std::pow(10.0, radarParams.loss / 10.0);  // 转换为线性值

    // 计算有效照射面积
    SignalType A_cell = calculateEffectiveArea(cell);

    // 获取 sigma0（后向散射系数）
    SignalType sigma0 = clutterParams.cellParams.sigma0;
    if (sigma0 == 0) {
        // 使用默认模型计算
        sigma0 = calculateSigma0(cell);
    }

    SignalType Pc = (Pt * G * G * lambda * lambda * sigma0 * A_cell) /
                    (std::pow(4 * M_PI, 3) * std::pow(R, 4) * L);

    return Pc;
}

SignalType GridManager::calculateEffectiveArea(const ClutterCell& cell) {
    // 有效照射面积 = 距离跨度 × 方位跨度 × 距离 × cos(掠射角)
    SignalType grazingAngle = calculateGrazingAngle(cell.range, m_antennaParams.height);

    SignalType azWidth = cell.range * cell.azimuthSpan;  // 方位宽度
    SignalType elWidth = cell.range * cell.elevationSpan; // 俯仰宽度

    // 对于地面/海面杂波，有效面积是方位宽度 × 距离分辨率
    SignalType A_cell = azWidth * cell.rangeSpan / std::cos(grazingAngle);

    return A_cell;
}

SignalType GridManager::calculateGrazingAngle(SignalType range, SignalType antennaHeight) {
    // 掠射角 = arctan(h/R)
    if (range < antennaHeight) {
        return M_PI / 2.0;  // 垂直向下
    }
    return std::atan(antennaHeight / range);
}

SignalType GridManager::calculateSigma0(const ClutterCell& cell) {
    // 简化的 Nathanson 海杂波模型
    // sigma0 = K * (grazing_angle)^gamma * (wind_speed)^delta

    SignalType grazingAngle = calculateGrazingAngle(cell.range, m_antennaParams.height);
    SignalType windSpeed = cell.clutterPower;  // 这里复用了字段，实际应该是风速

    // 典型参数（X 波段，HH 极化）
    SignalType K = 0.001;
    SignalType gamma = 1.5;
    SignalType delta = 2.0;

    SignalType sigma0 = K * std::pow(grazingAngle, gamma) * std::pow(windSpeed + 1.0, delta);

    return sigma0;
}

void GridManager::clear() {
    m_gridCache.clear();
    m_totalCells = 0;
}
