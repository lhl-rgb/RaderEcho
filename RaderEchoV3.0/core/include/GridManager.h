#pragma once

#include "Types.h"
#include <vector>
#include <string>
#include <memory>

class GridManager {
public:
    explicit GridManager();
    ~GridManager();

    void initialize(const RadarSystemParams& radarParams, const AntennaParams& antennaParams);

    std::vector<ClutterCell> generateGridForBeam(const BeamPosition& beamPos,
                                                  int numRangeBins,
                                                  int numAzBins = 1,
                                                  int numElBins = 1);

    SignalType calculateCellGain(const ClutterCell& cell, const BeamPosition& beamPos);

    SignalType calculateCellClutterPower(const ClutterCell& cell,
                                          const ClutterParams& clutterParams,
                                          const RadarSystemParams& radarParams);

    SignalType calculateEffectiveArea(const ClutterCell& cell);

    SignalType calculateGrazingAngle(SignalType range, SignalType antennaHeight);

    SignalType calculateSigma0(const ClutterCell& cell);

    size_t getTotalCells() const { return m_totalCells; }

    void clear();

private:
    RadarSystemParams m_radarParams;
    AntennaParams m_antennaParams;

    size_t m_totalCells = 0;
    std::vector<std::vector<ClutterCell>> m_gridCache;
};
