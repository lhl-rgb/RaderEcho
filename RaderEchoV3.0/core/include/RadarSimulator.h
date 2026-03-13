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

struct ScanData {
    std::vector<ComplexVector> pulseData;
    int scanIndex = 0;
    int beamIndex = 0;
    SignalType scanTime = 0;
    SignalType azimuth = 0;
    SignalType elevation = 0;
};

struct CPIData {
    ComplexMatrix echoMatrix;
    int cpiIndex = 0;
    int numPulses = 0;
    int numRangeBins = 0;
    int beamIndex = 0;
    SignalType startTime = 0;
    BeamPosition beamPos;
};

class PhasedArrayRadarSimulator {
public:
    PhasedArrayRadarSimulator();
    ~PhasedArrayRadarSimulator();

    bool initialize(const std::string& configPath);
    bool initialize(const SimulationConfig& config,
                   const RadarSystemParams& radarParams,
                   const AntennaParams& antennaParams,
                   const ClutterParams& clutterParams);
    void reset();
    void stop();

    size_t addTarget(const TargetParams& target);
    void removeTarget(size_t id);
    void updateTarget(size_t id, const TargetParams& target);
    std::vector<TargetParams> getAllTargets() const;

    ScanData runSingleScan();
    CPIData runSingleCPI(int beamIndex);
    void runMultipleScans(int numScans, const std::string& outputDir = "../out/data");
    void runFullSimulation();

    bool isRunning() const { return m_running; }
    bool isInitialized() const { return m_initialized; }
    int getCurrentScanIndex() const { return m_currentScan; }
    int getCurrentCPIIndex() const { return m_currentCPI; }

    const SimulationConfig& getConfig() const { return m_config; }
    const RadarSystemParams& getRadarParams() const { return m_radarParams; }
    const AntennaParams& getAntennaParams() const { return m_antennaParams; }
    const ClutterParams& getClutterParams() const { return m_clutterParams; }

private:
    void updateSimulationState(SignalType dt);
    void accumulateCellContribution(const ClutterCell& cell,
                                    const ComplexVector& clutterSeq,
                                    ComplexVector& rangeProfile);
    void saveScanData(const ScanData& data, const std::string& outputDir);
    void saveCPIData(const CPIData& data, const std::string& outputDir);
    bool loadBeamCodes(const std::string& filename);

private:
    SimulationConfig m_config;
    RadarSystemParams m_radarParams;
    AntennaParams m_antennaParams;
    ClutterParams m_clutterParams;
    SpectrumParams m_spectrumParams;

    std::unique_ptr<TargetManager> m_targetManager;
    std::unique_ptr<ClutterManager> m_clutterManager;
    std::unique_ptr<GridManager> m_gridManager;
    std::unique_ptr<SignalProcessor> m_signalProcessor;
    std::unique_ptr<BeamCodeManager> m_beamCodeManager;

    bool m_initialized = false;
    bool m_running = false;
    int m_currentScan = 0;
    int m_currentCPI = 0;
    int m_currentPulse = 0;
    SignalType m_currentTime = 0;

    ComplexVector m_referenceSignal;
    std::vector<ClutterCell> m_currentGrid;
};
