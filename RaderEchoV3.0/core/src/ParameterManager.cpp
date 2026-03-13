#include "ParameterManager.h"
#include "Logger.h"
#include <sstream>
#include <algorithm>
#include <cctype>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================================
// Beam Code Manager Implementation
// ============================================================================

BeamCodeManager::BeamCodeManager() = default;

BeamCodeManager::~BeamCodeManager() = default;

bool BeamCodeManager::loadFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        LOG_ERROR("Failed to open beam code file: " + filename);
        return false;
    }

    std::string content((std::istreambuf_iterator<char>(file)),
                        std::istreambuf_iterator<char>());
    file.close();

    return loadFromString(content);
}

bool BeamCodeManager::loadFromString(const std::string& content) {
    clear();

    std::istringstream iss(content);
    std::string line;

    while (std::getline(iss, line)) {
        // 跳过空行和注释
        if (line.empty() || line[0] == '#') continue;

        // 去除前后空白
        line.erase(0, line.find_first_not_of(" \t"));
        line.erase(line.find_last_not_of(" \t") + 1);
        if (line.empty()) continue;

        std::istringstream lineStream(line);
        BeamPosition beam;

        if (lineStream >> beam.beamCode >> beam.azimuth >> beam.elevation) {
            m_beams.push_back(beam);
        } else {
            LOG_WARN("Failed to parse beam code line: " + line);
        }
    }

    // 按波位码排序
    std::sort(m_beams.begin(), m_beams.end(),
              [](const BeamPosition& a, const BeamPosition& b) {
                  return a.beamCode < b.beamCode;
              });

    m_loaded = !m_beams.empty();
    LOG_INFO("Loaded " + std::to_string(m_beams.size()) + " beam positions");

    return m_loaded;
}

BeamPosition BeamCodeManager::getBeamAt(size_t index) const {
    if (index < m_beams.size()) {
        return m_beams[index];
    }
    return BeamPosition();
}

BeamPosition BeamCodeManager::nextBeam() {
    if (m_currentBeamIndex >= m_beams.size()) {
        m_currentBeamIndex = 0;  // 循环
    }
    return m_beams[m_currentBeamIndex++];
}

void BeamCodeManager::clear() {
    m_beams.clear();
    m_currentBeamIndex = 0;
    m_loaded = false;
}

// ============================================================================
// Parameter Manager Implementation
// ============================================================================

ParameterManager::ParameterManager() = default;

ParameterManager::~ParameterManager() = default;

SimulationConfig ParameterManager::loadFromFile(const std::string& filename) {
    SimulationConfig config = getDefaultConfig();
    RadarSystemParams radarParams = getDefaultRadarParams();
    AntennaParams antennaParams = getDefaultAntennaParams();
    ClutterParams clutterParams = getDefaultClutterParams();

    std::ifstream file(filename);
    if (!file.is_open()) {
        LOG_ERROR("Failed to open config file: " + filename);
        return config;
    }

    std::string line;
    while (std::getline(file, line)) {
        parseLine(line, config, radarParams, antennaParams, clutterParams);
    }

    file.close();

    // 更新推导参数
    updateDerivedParams(radarParams, antennaParams);

    LOG_INFO("Parameters loaded from: " + filename);

    return config;
}

void ParameterManager::parseLine(const std::string& line,
                                  SimulationConfig& config,
                                  RadarSystemParams& radarParams,
                                  AntennaParams& antennaParams,
                                  ClutterParams& clutterParams) {
    // 跳过空行和注释
    if (line.empty() || line[0] == '#') return;

    std::istringstream iss(line);
    std::string key;
    iss >> key;

    // 仿真配置
    if (key == "simulation_name") {
        iss >> config.name;
    } else if (key == "duration") {
        iss >> config.duration;
    } else if (key == "beam_code_file") {
        iss >> config.beamCodeFile;
    } else if (key == "output_file") {
        iss >> config.outputFile;
    } else if (key == "clutter_data_file") {
        iss >> config.clutterDataFile;
    } else if (key == "clutter_pool_size") {
        iss >> config.clutterPoolSize;
    } else if (key == "generate_clutter") {
        std::string val;
        iss >> val;
        config.generateClutter = (val == "true" || val == "1");
    } else if (key == "generate_targets") {
        std::string val;
        iss >> val;
        config.generateTargets = (val == "true" || val == "1");
    } else if (key == "add_noise") {
        std::string val;
        iss >> val;
        config.addNoise = (val == "true" || val == "1");

    // 雷达系统参数
    } else if (key == "frequency") {
        iss >> radarParams.frequency;
    } else if (key == "bandwidth") {
        iss >> radarParams.bandwidth;
    } else if (key == "prf") {
        iss >> radarParams.prf;
    } else if (key == "pulse_width") {
        iss >> radarParams.pulseWidth;
    } else if (key == "sample_rate") {
        iss >> radarParams.sampleRate;
    } else if (key == "peak_power") {
        iss >> radarParams.peakPower;
    } else if (key == "noise_figure") {
        iss >> radarParams.noiseFigure;
    } else if (key == "loss") {
        iss >> radarParams.loss;
    } else if (key == "max_range") {
        iss >> radarParams.maxRange;
    } else if (key == "num_pulses_per_cpi") {
        iss >> radarParams.numPulsesPerCPI;
    }

    // 天线参数
    } else if (key == "antenna_gain") {
        iss >> antennaParams.gain_dB;
    } else if (key == "antenna_height") {
        iss >> antennaParams.height;
    } else if (key == "az_beamwidth") {
        iss >> antennaParams.azBeamwidth_deg;
    } else if (key == "el_beamwidth") {
        iss >> antennaParams.elBeamwidth_deg;
    } else if (key == "num_elements_az") {
        iss >> antennaParams.numElementsAz;
    } else if (key == "num_elements_el") {
        iss >> antennaParams.numElementsEl;
    } else if (key == "element_spacing") {
        iss >> antennaParams.elementSpacing;
    } else if (key == "scan_rate") {
        iss >> antennaParams.scanRate;
    } else if (key == "polarization") {
        iss >> antennaParams.polarization;

    // 杂波参数
    } else if (key == "clutter_type") {
        std::string type;
        iss >> type;
        if (type == "Rayleigh") {
            clutterParams.type = ClutterDistributionType::Rayleigh;
        } else if (type == "Weibull") {
            clutterParams.type = ClutterDistributionType::Weibull;
        } else if (type == "LogNormal") {
            clutterParams.type = ClutterDistributionType::LogNormal;
        } else if (type == "K_Distribution") {
            clutterParams.type = ClutterDistributionType::K_Distribution;
        }
    } else if (key == "weibull_scale") {
        iss >> clutterParams.weibullScale;
    } else if (key == "weibull_shape") {
        iss >> clutterParams.weibullShape;
    } else if (key == "lognormal_mean") {
        iss >> clutterParams.logNormalMean;
    } else if (key == "lognormal_std") {
        iss >> clutterParams.logNormalStd;
    } else if (key == "kdist_shape") {
        iss >> clutterParams.kDistShape;
    } else if (key == "spectrum_center_freq") {
        iss >> clutterParams.spectrum.centerFreq;
    } else if (key == "spectrum_bandwidth") {
        iss >> clutterParams.spectrum.bandwidth;
    } else if (key == "sigma0") {
        iss >> clutterParams.cellParams.sigma0;
    } else if (key == "wind_speed") {
        iss >> clutterParams.cellParams.windSpeed;
    }
}

void ParameterManager::updateDerivedParams(RadarSystemParams& radarParams,
                                            AntennaParams& antennaParams) {
    // 更新雷达推导参数
    radarParams.updateDerivedParams();

    // 更新天线推导参数
    antennaParams.updateDerivedParams(radarParams.frequency);
}

bool ParameterManager::saveToFile(const std::string& filename,
                                   const SimulationConfig& config,
                                   const RadarSystemParams& radarParams,
                                   const AntennaParams& antennaParams,
                                   const ClutterParams& clutterParams) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        LOG_ERROR("Failed to create config file: " + filename);
        return false;
    }

    file << "# Radar Echo Simulator Configuration File\n\n";

    file << "# ============================================================================\n";
    file << "# Simulation Configuration\n";
    file << "# ============================================================================\n";
    file << "simulation_name " << config.name << "\n";
    file << "duration " << config.duration << "\n";
    file << "beam_code_file " << config.beamCodeFile << "\n";
    file << "output_file " << config.outputFile << "\n";
    file << "clutter_data_file " << config.clutterDataFile << "\n";
    file << "clutter_pool_size " << config.clutterPoolSize << "\n";
    file << "generate_clutter " << (config.generateClutter ? "true" : "false") << "\n";
    file << "generate_targets " << (config.generateTargets ? "true" : "false") << "\n";
    file << "add_noise " << (config.addNoise ? "true" : "false") << "\n\n";

    file << "# ============================================================================\n";
    file << "# Radar System Parameters\n";
    file << "# ============================================================================\n";
    file << "frequency " << radarParams.frequency << "\n";
    file << "bandwidth " << radarParams.bandwidth << "\n";
    file << "prf " << radarParams.prf << "\n";
    file << "pulse_width " << radarParams.pulseWidth << "\n";
    file << "sample_rate " << radarParams.sampleRate << "\n";
    file << "peak_power " << radarParams.peakPower << "\n";
    file << "noise_figure " << radarParams.noiseFigure << "\n";
    file << "loss " << radarParams.loss << "\n";
    file << "max_range " << radarParams.maxRange << "\n";
    file << "num_pulses_per_cpi " << radarParams.numPulsesPerCPI << "\n\n";

    file << "# ============================================================================\n";
    file << "# Antenna Parameters\n";
    file << "# ============================================================================\n";
    file << "antenna_gain " << antennaParams.gain_dB << "\n";
    file << "antenna_height " << antennaParams.height << "\n";
    file << "az_beamwidth " << antennaParams.azBeamwidth_deg << "\n";
    file << "el_beamwidth " << antennaParams.elBeamwidth_deg << "\n";
    file << "num_elements_az " << antennaParams.numElementsAz << "\n";
    file << "num_elements_el " << antennaParams.numElementsEl << "\n";
    file << "element_spacing " << antennaParams.elementSpacing << "\n";
    file << "scan_rate " << antennaParams.scanRate << "\n";
    file << "polarization " << antennaParams.polarization << "\n\n";

    file << "# ============================================================================\n";
    file << "# Clutter Parameters\n";
    file << "# ============================================================================\n";

    std::string typeStr;
    switch (clutterParams.type) {
        case ClutterDistributionType::Rayleigh: typeStr = "Rayleigh"; break;
        case ClutterDistributionType::Weibull: typeStr = "Weibull"; break;
        case ClutterDistributionType::LogNormal: typeStr = "LogNormal"; break;
        case ClutterDistributionType::K_Distribution: typeStr = "K_Distribution"; break;
    }
    file << "clutter_type " << typeStr << "\n";
    file << "weibull_scale " << clutterParams.weibullScale << "\n";
    file << "weibull_shape " << clutterParams.weibullShape << "\n";
    file << "lognormal_mean " << clutterParams.logNormalMean << "\n";
    file << "lognormal_std " << clutterParams.logNormalStd << "\n";
    file << "kdist_shape " << clutterParams.kDistShape << "\n";
    file << "spectrum_center_freq " << clutterParams.spectrum.centerFreq << "\n";
    file << "spectrum_bandwidth " << clutterParams.spectrum.bandwidth << "\n";
    file << "sigma0 " << clutterParams.cellParams.sigma0 << "\n";
    file << "wind_speed " << clutterParams.cellParams.windSpeed << "\n";

    file.close();
    LOG_INFO("Parameters saved to: " + filename);

    return true;
}

SimulationConfig ParameterManager::getDefaultConfig() const {
    SimulationConfig config;
    config.name = "Default Simulation";
    config.duration = 10.0;
    config.beamCodeFile = "../config/beam_codes.txt";
    config.outputFile = "../out/data/echo_data.txt";
    config.clutterDataFile = "../out/data/clutter_data.bin";
    config.clutterPoolSize = 100;
    config.generateClutter = true;
    config.generateTargets = true;
    config.addNoise = true;
    return config;
}

RadarSystemParams ParameterManager::getDefaultRadarParams() const {
    RadarSystemParams params;
    params.frequency = 9.4e9;        // X 波段 9.4 GHz
    params.bandwidth = 20e6;         // 20 MHz
    params.prf = 1200;               // 1200 Hz
    params.pulseWidth = 0.5e-6;      // 0.5 us
    params.sampleRate = 60e6;        // 60 MHz
    params.peakPower = 25e3;         // 25 kW
    params.noiseFigure = 3.0;        // 3 dB
    params.loss = 2.0;               // 2 dB
    params.maxRange = 20e3;          // 20 km
    params.numPulsesPerCPI = 64;     // 64 脉冲/CPI
    params.updateDerivedParams();
    return params;
}

AntennaParams ParameterManager::getDefaultAntennaParams() const {
    AntennaParams params;
    params.gain_dB = 30.0;           // 30 dB
    params.height = 15.0;            // 15 m
    params.azBeamwidth_deg = 1.5;    // 1.5 度
    params.elBeamwidth_deg = 20.0;   // 20 度
    params.numElementsAz = 32;       // 32 阵元
    params.numElementsEl = 8;        // 8 阵元
    params.scanRate = 24;            // 24 rpm
    params.polarization = "HH";
    return params;
}

ClutterParams ParameterManager::getDefaultClutterParams() const {
    ClutterParams params;
    params.type = ClutterDistributionType::K_Distribution;
    params.weibullScale = 1.0;
    params.weibullShape = 1.5;
    params.logNormalMean = 0.0;
    params.logNormalStd = 1.0;
    params.kDistShape = 5.0;
    params.spectrum.centerFreq = 200.0;
    params.spectrum.bandwidth = 100.0;
    params.cellParams.sigma0 = 0.01;
    params.cellParams.windSpeed = 5.0;
    return params;
}

bool ParameterManager::validateParams(const SimulationConfig& config,
                                       const RadarSystemParams& radarParams,
                                       const AntennaParams& antennaParams) {
    bool valid = true;

    if (radarParams.frequency <= 0) {
        LOG_WARN("Invalid frequency: must be positive");
        valid = false;
    }
    if (radarParams.prf <= 0) {
        LOG_WARN("Invalid PRF: must be positive");
        valid = false;
    }
    if (radarParams.sampleRate <= 0) {
        LOG_WARN("Invalid sample rate: must be positive");
        valid = false;
    }
    if (radarParams.maxRange <= 0) {
        LOG_WARN("Invalid max range: must be positive");
        valid = false;
    }
    if (antennaParams.gain_dB <= 0) {
        LOG_WARN("Invalid antenna gain: must be positive");
        valid = false;
    }

    return valid;
}
