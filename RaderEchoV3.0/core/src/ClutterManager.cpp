#include "ClutterManager.h"
#include "Logger.h"
#include <fstream>
#include <cstring>

ClutterMemoryPool::ClutterMemoryPool(size_t poolSize, size_t sequenceLength)
    : m_poolSize(poolSize), m_sequenceLength(sequenceLength) {
    m_blocks.resize(poolSize);
    m_dataPool.resize(poolSize);
}

ClutterMemoryPool::~ClutterMemoryPool() = default;

void ClutterMemoryPool::initialize(const ClutterParams& params,
                                    const SpectrumParams& spectrumParams,
                                    bool useSIRP) {
    std::lock_guard<std::mutex> lock(m_mutex);

    m_cachedParams = params;
    m_cachedSpectrumParams = spectrumParams;

    LOG_INFO("Initializing clutter memory pool...");

    for (size_t i = 0; i < m_poolSize; ++i) {
        m_blocks[i].id = i;
        m_blocks[i].length = m_sequenceLength;
        m_blocks[i].type = params.type;

        if (useSIRP) {
            auto sirpGen = std::make_unique<SIRPGenerator>(params.type);
            m_blocks[i].data = sirpGen->generate(m_sequenceLength, params, spectrumParams);
        } else {
            auto zmnlGen = std::make_unique<ZMNLGenerator>(params.type);
            m_blocks[i].data = zmnlGen->generate(m_sequenceLength, params);
        }
        m_dataPool[i] = m_blocks[i].data;
    }

    m_initialized = true;
    LOG_INFO("Memory pool initialization complete");
}

const ComplexVector& ClutterMemoryPool::getSequence(size_t index, size_t length) {
    if (!m_initialized) {
        static ComplexVector empty;
        return empty;
    }

    index = index % m_poolSize;
    (void)length;

    return m_dataPool[index];
}

const ComplexVector* ClutterMemoryPool::getRandomSequence(size_t offset) {
    if (!m_initialized) {
        return nullptr;
    }

    size_t index = m_accessIndex.fetch_add(1) % m_poolSize;
    (void)offset;
    return &m_dataPool[index];
}

bool ClutterMemoryPool::saveToFile(const std::string& filename) {
    if (!m_initialized) {
        LOG_ERROR("Memory pool not initialized");
        return false;
    }

    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        LOG_ERROR("Failed to open file: " + filename);
        return false;
    }

    size_t header[4] = {m_poolSize, m_sequenceLength,
                        static_cast<size_t>(m_cachedParams.type), sizeof(Complex)};
    file.write(reinterpret_cast<char*>(header), sizeof(header));

    for (const auto& block : m_dataPool) {
        file.write(reinterpret_cast<const char*>(block.data()),
                   block.size() * sizeof(Complex));
    }

    file.close();
    LOG_INFO("Saved clutter data to: " + filename);
    return true;
}

bool ClutterMemoryPool::loadFromFile(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        LOG_ERROR("Failed to open file: " + filename);
        return false;
    }

    size_t header[4];
    file.read(reinterpret_cast<char*>(header), sizeof(header));

    size_t poolSize = header[0];
    size_t seqLength = header[1];

    if (poolSize != m_poolSize || seqLength != m_sequenceLength) {
        m_poolSize = poolSize;
        m_sequenceLength = seqLength;
        m_blocks.resize(poolSize);
        m_dataPool.resize(poolSize);
    }

    for (size_t i = 0; i < poolSize; ++i) {
        m_blocks[i].id = i;
        m_blocks[i].length = seqLength;
        m_dataPool[i].resize(seqLength);
        file.read(reinterpret_cast<char*>(m_dataPool[i].data()),
                  seqLength * sizeof(Complex));
    }

    file.close();
    m_initialized = true;
    LOG_INFO("Loaded clutter data from: " + filename);
    return true;
}

// ============================================================================
// Clutter Manager Implementation
// ============================================================================

ClutterManager::ClutterManager() : m_memoryPool(std::make_unique<ClutterMemoryPool>()) {}

ClutterManager::~ClutterManager() = default;

void ClutterManager::initialize(const ClutterParams& params,
                                 const SpectrumParams& spectrumParams,
                                 size_t poolSize,
                                 size_t sequenceLength) {
    m_params = params;
    m_spectrumParams = spectrumParams;

    m_memoryPool = std::make_unique<ClutterMemoryPool>(poolSize, sequenceLength);
    m_memoryPool->initialize(params, spectrumParams, m_useSIRP);

    if (m_useSIRP) {
        m_sirpGenerator = std::make_unique<SIRPGenerator>(params.type);
    } else {
        m_zmnlGenerator = std::make_unique<ZMNLGenerator>(params.type);
    }

    LOG_INFO("Clutter manager initialized");
}

ComplexMatrix ClutterManager::generateCellClutter(const ClutterCell& cell,
                                                   int numPulses) {
    ComplexMatrix clutterMatrix(numPulses);

    size_t baseIndex = cell.sequenceIndex;

    for (int pulseIdx = 0; pulseIdx < numPulses; ++pulseIdx) {
        size_t seqIndex = (baseIndex + pulseIdx) % m_memoryPool->getPoolSize();
        const ComplexVector* seq = &m_memoryPool->getSequence(seqIndex,
                                                             m_memoryPool->getSequenceLength());

        if (seq && !seq->empty()) {
            clutterMatrix[pulseIdx] = *seq;
        } else {
            if (m_useSIRP && m_sirpGenerator) {
                clutterMatrix[pulseIdx] = m_sirpGenerator->generate(
                    m_memoryPool->getSequenceLength(), m_params, m_spectrumParams);
            } else if (m_zmnlGenerator) {
                clutterMatrix[pulseIdx] = m_zmnlGenerator->generate(
                    m_memoryPool->getSequenceLength(), m_params);
            }
        }
    }

    return clutterMatrix;
}

ComplexVector ClutterManager::getModulatedClutter(const ClutterCell& cell,
                                                   int pulseIndex) {
    size_t seqIndex = (cell.sequenceIndex + pulseIndex) % m_memoryPool->getPoolSize();
    const ComplexVector* baseClutter = &m_memoryPool->getSequence(seqIndex,
                                                                 m_memoryPool->getSequenceLength());

    if (!baseClutter || baseClutter->empty()) {
        return ComplexVector();
    }

    ComplexVector modulatedClutter = *baseClutter;
    SignalType gainScale = std::sqrt(cell.antennaGain);

    for (auto& sample : modulatedClutter) {
        sample *= gainScale;
    }

    return modulatedClutter;
}

bool ClutterManager::saveClutterData(const std::string& filename) {
    return m_memoryPool->saveToFile(filename);
}

bool ClutterManager::loadClutterData(const std::string& filename) {
    return m_memoryPool->loadFromFile(filename);
}
