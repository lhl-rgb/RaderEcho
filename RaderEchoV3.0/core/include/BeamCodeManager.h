#pragma once

#include "Types.h"
#include <vector>
#include <string>
#include <fstream>

/**
 * @brief 波位码管理器
 * @details 负责波位码文件的读取、解析和波束序列管理
 */
class BeamCodeManager {
public:
    BeamCodeManager();
    ~BeamCodeManager();

    bool loadFromFile(const std::string& filename);
    bool loadFromString(const std::string& content);
    BeamPosition getBeamAt(size_t index) const;
    const std::vector<BeamPosition>& getAllBeams() const { return m_beams; }
    size_t getBeamCount() const { return m_beams.size(); }
    size_t getCurrentBeamIndex() const { return m_currentBeamIndex; }
    BeamPosition nextBeam();
    void reset() { m_currentBeamIndex = 0; }
    void setCurrentBeamIndex(size_t index) { m_currentBeamIndex = index; }
    void clear();

private:
    std::vector<BeamPosition> m_beams;
    size_t m_currentBeamIndex = 0;
    bool m_loaded = false;
};
