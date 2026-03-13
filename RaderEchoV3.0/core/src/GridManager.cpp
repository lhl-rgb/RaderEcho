#include "GridManager.h"
#include "Logger.h"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

    SignalType azSpan = azBeamwidth_rad / numAzBins;
    SignalType elSpan = elBeamwidth_rad / numElBins;
    SignalType rangeSpan = m_radarParams.rangeBinSize;

    for (int r = 0; r < numRangeBins; ++r) {
        for (int az = 0; az < numAzBins; ++az) {
            for (int el = 0; el < numElBins; ++el) {
                ClutterCell cell;

                cell.rangeIndex = r;
                cell.range = m_radarParams.minRange + r * rangeSpan + rangeSpan / 2.0;
                cell.rangeSpan = rangeSpan;

                cell.azimuthIndex = az;
                cell.azimuth = beamPos.azimuth * M_PI / 180.0 +
                               (az - numAzBins / 2.0) * azSpan;
                cell.azimuthSpan = azSpan;

                cell.elevationIndex = el;
                cell.elevation = beamPos.elevation * M_PI / 180.0 +
                                 (el - numElBins / 2.0) * elSpan;
                cell.elevationSpan = elSpan;

                cell.antennaGain = calculateCellGain(cell, beamPos);
                cell.clutterPower = cell.range;

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

    SignalType gain = m_antennaParams.calculateArrayGain(
        cell.azimuth, cell.elevation, azSteer, elSteer);

    gain *= m_antennaParams.gain_linear;

    return gain;
}

SignalType GridManager::calculateCellClutterPower(const ClutterCell& cell,
                                                   const ClutterParams& clutterParams,
                                                   const RadarSystemParams& radarParams) {
    SignalType Pt = radarParams.peakPower;
    SignalType G = cell.antennaGain;
    SignalType lambda = radarParams.wavelength;
    SignalType R = cell.range;
    SignalType L = std::pow(10.0, radarParams.loss / 10.0);

    SignalType A_cell = calculateEffectiveArea(cell);

    SignalType sigma0 = clutterParams.cellParams.sigma0;
    if (sigma0 == 0) {
        sigma0 = calculateSigma0(cell);
    }

    SignalType Pc = (Pt * G * G * lambda * lambda * sigma0 * A_cell) /
                    (std::pow(4 * M_PI, 3) * std::pow(R, 4) * L);

    return Pc;
}

SignalType GridManager::calculateEffectiveArea(const ClutterCell& cell) {
    SignalType grazingAngle = calculateGrazingAngle(cell.range, m_antennaParams.height);

    SignalType azWidth = cell.range * cell.azimuthSpan;
    SignalType A_cell = azWidth * cell.rangeSpan / std::cos(grazingAngle);

    return A_cell;
}

SignalType GridManager::calculateGrazingAngle(SignalType range, SignalType antennaHeight) {
    if (range < antennaHeight) {
        return M_PI / 2.0;
    }
    return std::atan(antennaHeight / range);
}

SignalType GridManager::calculateSigma0(const ClutterCell& cell) {
    SignalType grazingAngle = calculateGrazingAngle(cell.range, m_antennaParams.height);
    SignalType windSpeed = 5.0;

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
