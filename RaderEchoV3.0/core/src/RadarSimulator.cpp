#include "RadarSimulator.h"
#include "Logger.h"
#include <fstream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <sstream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================================
// Phased Array Radar Simulator Implementation
// ============================================================================

PhasedArrayRadarSimulator::PhasedArrayRadarSimulator() = default;

PhasedArrayRadarSimulator::~PhasedArrayRadarSimulator() {
    stop();
}

bool PhasedArrayRadarSimulator::initialize(const std::string& configPath) {
    ParameterManager paramMgr;
    m_config = paramMgr.loadFromFile(configPath);

    // 获取默认参数
    m_radarParams = paramMgr.getDefaultRadarParams();
    m_antennaParams = paramMgr.getDefaultAntennaParams();
    m_clutterParams = paramMgr.getDefaultClutterParams();

    m_spectrumParams.centerFreq = m_clutterParams.spectrum.centerFreq;
    m_spectrumParams.bandwidth = m_clutterParams.spectrum.bandwidth;

    // 初始化功能模块
    m_targetManager = std::make_unique<TargetManager>();
    m_clutterManager = std::make_unique<ClutterManager>();
    m_gridManager = std::make_unique<GridManager>();
    m_signalProcessor = std::make_unique<SignalProcessor>();
    m_beamCodeManager = std::make_unique<BeamCodeManager>();

    // 初始化信号处理器
    m_signalProcessor->initialize(m_radarParams, m_antennaParams);

    // 初始化网格管理器
    m_gridManager->initialize(m_radarParams, m_antennaParams);

    // 初始化杂波管理器
    m_clutterManager->initialize(m_clutterParams, m_spectrumParams,
                                  m_config.clutterPoolSize,
                                  m_radarParams.numRangeBins);

    // 加载波位码
    if (!m_config.beamCodeFile.empty()) {
        loadBeamCodes(m_config.beamCodeFile);
    }

    // 生成参考信号
    m_referenceSignal = m_signalProcessor->generateLFMWaveform(
        m_radarParams.pulseWidth,
        m_radarParams.bandwidth,
        m_radarParams.sampleRate);

    m_initialized = true;
    LOG_INFO("PhasedArrayRadarSimulator initialized successfully");

    return true;
}

bool PhasedArrayRadarSimulator::initialize(const SimulationConfig& config,
                                            const RadarSystemParams& radarParams,
                                            const AntennaParams& antennaParams,
                                            const ClutterParams& clutterParams) {
    m_config = config;
    m_radarParams = radarParams;
    m_antennaParams = antennaParams;
    m_clutterParams = clutterParams;

    m_spectrumParams.centerFreq = clutterParams.spectrum.centerFreq;
    m_spectrumParams.bandwidth = clutterParams.spectrum.bandwidth;

    // 初始化功能模块
    m_targetManager = std::make_unique<TargetManager>();
    m_clutterManager = std::make_unique<ClutterManager>();
    m_gridManager = std::make_unique<GridManager>();
    m_signalProcessor = std::make_unique<SignalProcessor>();
    m_beamCodeManager = std::make_unique<BeamCodeManager>();

    // 初始化信号处理器
    m_signalProcessor->initialize(m_radarParams, m_antennaParams);

    // 初始化网格管理器
    m_gridManager->initialize(m_radarParams, m_antennaParams);

    // 初始化杂波管理器
    m_clutterManager->initialize(m_clutterParams, m_spectrumParams,
                                  m_config.clutterPoolSize,
                                  m_radarParams.numRangeBins);

    // 加载波位码
    if (!m_config.beamCodeFile.empty()) {
        loadBeamCodes(m_config.beamCodeFile);
    }

    // 生成参考信号
    m_referenceSignal = m_signalProcessor->generateLFMWaveform(
        m_radarParams.pulseWidth,
        m_radarParams.bandwidth,
        m_radarParams.sampleRate);

    m_initialized = true;
    LOG_INFO("PhasedArrayRadarSimulator initialized successfully");

    return true;
}

void PhasedArrayRadarSimulator::reset() {
    m_currentScan = 0;
    m_currentCPI = 0;
    m_currentPulse = 0;
    m_currentTime = 0;
    m_running = false;

    if (m_beamCodeManager) {
        m_beamCodeManager->reset();
    }
    if (m_gridManager) {
        m_gridManager->clear();
    }

    LOG_INFO("Simulator reset");
}

void PhasedArrayRadarSimulator::stop() {
    m_running = false;
    LOG_INFO("Simulator stopped");
}

size_t PhasedArrayRadarSimulator::addTarget(const TargetParams& target) {
    if (!m_targetManager) {
        return 0;
    }
    return m_targetManager->addTarget(target);
}

void PhasedArrayRadarSimulator::removeTarget(size_t id) {
    if (m_targetManager) {
        m_targetManager->removeTarget(id);
    }
}

void PhasedArrayRadarSimulator::updateTarget(size_t id, const TargetParams& target) {
    if (m_targetManager) {
        m_targetManager->updateTarget(id, target);
    }
}

std::vector<TargetParams> PhasedArrayRadarSimulator::getAllTargets() const {
    if (!m_targetManager) {
        return {};
    }
    return m_targetManager->getAllTargets();
}

ScanData PhasedArrayRadarSimulator::runSingleScan() {
    ScanData scanData;
    scanData.scanIndex = m_currentScan;
    scanData.scanTime = m_currentTime;

    if (!m_beamCodeManager || m_beamCodeManager->getBeamCount() == 0) {
        LOG_ERROR("No beam codes loaded");
        return scanData;
    }

    // 对每个波束位置执行扫描
    BeamPosition beamPos = m_beamCodeManager->nextBeam();
    scanData.azimuth = beamPos.azimuth;
    scanData.elevation = beamPos.elevation;

    // 生成该波束的网格
    m_currentGrid = m_gridManager->generateGridForBeam(
        beamPos, m_radarParams.numRangeBins, 1, 1);

    // 生成 CPI 数据
    CPIData cpiData = runSingleCPI(m_beamCodeManager->getCurrentBeamIndex() - 1);

    // 将 CPI 数据转换为扫描数据
    scanData.pulseData = std::move(cpiData.echoMatrix);
    scanData.beamIndex = cpiData.beamIndex;

    m_currentScan++;
    m_currentTime += m_radarParams.cpiDuration;

    return scanData;
}

CPIData PhasedArrayRadarSimulator::runSingleCPI(int beamIndex) {
    CPIData cpiData;
    cpiData.cpiIndex = m_currentCPI;
    cpiData.numPulses = m_radarParams.numPulsesPerCPI;
    cpiData.numRangeBins = m_radarParams.numRangeBins;
    cpiData.startTime = m_currentTime;
    cpiData.beamPos = m_beamCodeManager->getBeamAt(beamIndex);

    // 初始化回波矩阵
    cpiData.echoMatrix.resize(m_radarParams.numRangeBins);
    for (int r = 0; r < m_radarParams.numRangeBins; ++r) {
        cpiData.echoMatrix[r].resize(m_radarParams.numPulsesPerCPI, Complex(0, 0));
    }

    // 为每个脉冲生成回波
    bool cpiStart = true;

    for (int pulseIdx = 0; pulseIdx < m_radarParams.numPulsesPerCPI; ++pulseIdx) {
        ComplexVector pulseEcho(m_radarParams.numRangeBins, Complex(0, 0));

        // 1. 生成杂波回波（网格累加）
        if (m_config.generateClutter) {
            for (const auto& cell : m_currentGrid) {
                // 获取调制后的杂波序列
                ComplexVector clutterSeq = m_clutterManager->getModulatedClutter(
                    cell, pulseIdx);

                // 累加到对应的距离门
                if (!clutterSeq.empty()) {
                    size_t rangeIdx = cell.rangeIndex;
                    if (rangeIdx < pulseEcho.size()) {
                        // 对杂波序列求平均作为该距离门的值
                        Complex avgClutter(0, 0);
                        for (const auto& sample : clutterSeq) {
                            avgClutter += sample;
                        }
                        avgClutter /= clutterSeq.size();
                        pulseEcho[rangeIdx] += avgClutter;
                    }
                }
            }
        }

        // 2. 生成目标回波
        if (m_config.generateTargets && m_targetManager) {
            auto targets = m_targetManager->getAllTargets();
            for (auto& target : targets) {
                // 更新目标 RCS
                m_targetManager->updateTargetRCS(target, cpiStart);

                // 生成目标回波
                ComplexVector targetEcho = m_signalProcessor->generateTargetEcho(
                    target, pulseIdx, m_currentTime);

                // 累加到脉冲回波
                for (size_t i = 0; i < targetEcho.size() && i < pulseEcho.size(); ++i) {
                    pulseEcho[i] += targetEcho[i];
                }
            }
        }

        // 3. 添加噪声
        if (m_config.addNoise) {
            SignalType noisePower = m_signalProcessor->calculateNoisePower(
                m_radarParams.bandwidth, m_radarParams.noiseFigure);
            ComplexVector noise = m_signalProcessor->generateThermalNoise(
                m_radarParams.numRangeBins, noisePower);

            for (size_t i = 0; i < pulseEcho.size(); ++i) {
                pulseEcho[i] += noise[i];
            }
        }

        // 存储脉冲数据
        for (int r = 0; r < m_radarParams.numRangeBins; ++r) {
            cpiData.echoMatrix[r][pulseIdx] = pulseEcho[r];
        }

        // 更新时间
        m_currentTime += m_radarParams.pri;
        m_currentPulse++;

        cpiStart = false;  // 只有第一个脉冲是 CPI 开始
    }

    m_currentCPI++;

    // 更新目标位置
    if (m_targetManager) {
        m_targetManager->updateAllTargets(m_radarParams.cpiDuration, m_currentTime);
    }

    return cpiData;
}

void PhasedArrayRadarSimulator::runMultipleScans(int numScans, const std::string& outputDir) {
    LOG_INFO("Starting multiple scans: " + std::to_string(numScans));

    for (int i = 0; i < numScans && m_running; ++i) {
        ScanData scanData = runSingleScan();
        saveScanData(scanData, outputDir);

        LOG_INFO_SIMPLE("Scan " + std::to_string(i + 1) + "/" +
                       std::to_string(numScans) + " completed");
    }

    LOG_INFO("Multiple scans completed");
}

void PhasedArrayRadarSimulator::runFullSimulation() {
    LOG_INFO("Starting full simulation");

    m_running = true;

    if (m_beamCodeManager) {
        int numBeams = static_cast<int>(m_beamCodeManager->getBeamCount());
        int numScansPerSweep = numBeams;

        // 计算总扫描次数
        SignalType sweepTime = numScansPerSweep * m_radarParams.cpiDuration;
        int totalSweeps = static_cast<int>(m_config.duration / sweepTime);

        for (int sweep = 0; sweep < totalSweeps && m_running; ++sweep) {
            m_beamCodeManager->reset();

            for (int beamIdx = 0; beamIdx < numBeams && m_running; ++beamIdx) {
                CPIData cpiData = runSingleCPI(beamIdx);
                saveCPIData(cpiData, "../out/data");
            }

            LOG_INFO_SIMPLE("Sweep " + std::to_string(sweep + 1) + "/" +
                           std::to_string(totalSweeps) + " completed");
        }
    }

    m_running = false;
    LOG_INFO("Full simulation completed");
}

void PhasedArrayRadarSimulator::updateSimulationState(SignalType dt) {
    (void)dt;
    // 更新仿真状态（可用于实时更新目标位置等）
}

void PhasedArrayRadarSimulator::accumulateCellContribution(
    const ClutterCell& cell,
    const ComplexVector& clutterSeq,
    ComplexVector& rangeProfile) {

    if (clutterSeq.empty()) return;

    size_t rangeIdx = cell.rangeIndex;
    if (rangeIdx >= rangeProfile.size()) return;

    // 计算单元贡献（平均）
    Complex contribution(0, 0);
    for (const auto& sample : clutterSeq) {
        contribution += sample;
    }
    contribution /= clutterSeq.size();

    rangeProfile[rangeIdx] += contribution;
}

void PhasedArrayRadarSimulator::saveScanData(const ScanData& data,
                                              const std::string& outputDir) {
    std::ostringstream filename;
    filename << outputDir << "/scan_" << std::setfill('0') << std::setw(4)
             << data.scanIndex << ".txt";

    std::ofstream file(filename.str());
    if (!file.is_open()) {
        LOG_ERROR("Failed to open file: " + filename.str());
        return;
    }

    file << "# Scan Data\n";
    file << "# Scan Index: " << data.scanIndex << "\n";
    file << "# Beam Index: " << data.beamIndex << "\n";
    file << "# Azimuth: " << data.azimuth << " deg\n";
    file << "# Elevation: " << data.elevation << " deg\n";
    file << "# Time: " << data.scanTime << " s\n";
    file << "# Num Pulses: " << data.pulseData.size() << "\n\n";

    for (size_t pulseIdx = 0; pulseIdx < data.pulseData.size(); ++pulseIdx) {
        file << "# Pulse " << pulseIdx << "\n";
        for (const auto& sample : data.pulseData[pulseIdx]) {
            file << std::scientific << std::abs(sample) << " "
                 << std::arg(sample) << "\n";
        }
        file << "\n";
    }

    file.close();
    LOG_INFO_SIMPLE("Saved scan data to: " + filename.str());
}

void PhasedArrayRadarSimulator::saveCPIData(const CPIData& data,
                                             const std::string& outputDir) {
    std::ostringstream filename;
    filename << outputDir << "/cpi_" << std::setfill('0') << std::setw(4)
             << data.cpiIndex << ".bin";

    std::ofstream file(filename.str(), std::ios::binary);
    if (!file.is_open()) {
        LOG_ERROR("Failed to open file: " + filename.str());
        return;
    }

    // 写入文件头
    int32_t header[5] = {
        static_cast<int32_t>(data.cpiIndex),
        static_cast<int32_t>(data.numPulses),
        static_cast<int32_t>(data.numRangeBins),
        sizeof(Complex),
        static_cast<int32_t>(data.beamPos.beamCode)
    };
    file.write(reinterpret_cast<char*>(header), sizeof(header));

    // 写入波束信息
    file.write(reinterpret_cast<const char*>(&data.beamPos.azimuth), sizeof(SignalType));
    file.write(reinterpret_cast<const char*>(&data.beamPos.elevation), sizeof(SignalType));

    // 写入回波数据
    for (const auto& pulse : data.echoMatrix) {
        file.write(reinterpret_cast<const char*>(pulse.data()),
                   pulse.size() * sizeof(Complex));
    }

    file.close();
    LOG_INFO_SIMPLE("Saved CPI data to: " + filename.str());
}

bool PhasedArrayRadarSimulator::loadBeamCodes(const std::string& filename) {
    if (!m_beamCodeManager) {
        m_beamCodeManager = std::make_unique<BeamCodeManager>();
    }
    return m_beamCodeManager->loadFromFile(filename);
}
