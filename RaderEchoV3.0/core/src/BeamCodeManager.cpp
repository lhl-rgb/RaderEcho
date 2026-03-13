#include "BeamCodeManager.h"
#include "Logger.h"
#include <sstream>
#include <algorithm>

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
        if (line.empty() || line[0] == '#') continue;

        line.erase(0, line.find_first_not_of(" \t"));
        line.erase(line.find_last_not_of(" \t") + 1);
        if (line.empty()) continue;

        std::istringstream lineStream(line);
        BeamPosition beam;

        if (lineStream >> beam.beamCode >> beam.azimuth >> beam.elevation) {
            m_beams.push_back(beam);
        }
    }

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
        m_currentBeamIndex = 0;
    }
    return m_beams[m_currentBeamIndex++];
}

void BeamCodeManager::clear() {
    m_beams.clear();
    m_currentBeamIndex = 0;
    m_loaded = false;
}
