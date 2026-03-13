#pragma once

#include "Types.h"
#include "ClutterGenerator.h"
#include <vector>
#include <string>
#include <mutex>
#include <atomic>
#include <unordered_map>

/**
 * @brief 杂波数据块（存储在内存池中）
 */
struct ClutterBlock {
    size_t id = 0;                   // 块 ID
    size_t length = 0;               // 序列长度
    ClutterDistributionType type;    // 分布类型
    ComplexVector data;              // 杂波数据

    // 用于快速索引
    size_t offset = 0;               // 在内存池中的偏移
};

/**
 * @brief 杂波内存池管理器
 * @details 预生成杂波序列并存储在内存池中，供回波生成时快速访问
 *
 * 内存池结构：
 * - 按分布类型分区
 * - 每个分区包含多个固定长度的杂波块
 * - 支持随机索引和顺序访问
 */
class ClutterMemoryPool {
public:
    /**
     * @brief 构造函数
     * @param poolSize 池中杂波块数量
     * @param sequenceLength 每个序列的长度
     */
    explicit ClutterMemoryPool(size_t poolSize = 100, size_t sequenceLength = 1024);
    ~ClutterMemoryPool();

    /**
     * @brief 初始化内存池（预生成杂波）
     * @param params 杂波参数
     * @param spectrumParams 频谱参数
     * @param useSIRP 是否使用 SIRP 方法（否则用 ZMNL）
     */
    void initialize(const ClutterParams& params, const SpectrumParams& spectrumParams, bool useSIRP = true);

    /**
     * @brief 从内存池获取杂波序列
     * @param index 索引位置
     * @param length 需要的长度
     * @return 杂波序列（引用）
     */
    const ComplexVector& getSequence(size_t index, size_t length);

    /**
     * @brief 随机获取一个杂波序列
     * @param offset 偏移量（用于 CPI 内的相关性）
     * @return 杂波序列指针
     */
    const ComplexVector* getRandomSequence(size_t offset = 0);

    /**
     * @brief 获取池大小
     */
    size_t getPoolSize() const { return m_poolSize; }

    /**
     * @brief 获取序列长度
     */
    size_t getSequenceLength() const { return m_sequenceLength; }

    /**
     * @brief 将杂波数据保存到文件
     * @param filename 文件路径
     */
    bool saveToFile(const std::string& filename);

    /**
     * @brief 从文件加载杂波数据
     * @param filename 文件路径
     */
    bool loadFromFile(const std::string& filename);

private:
    /**
     * @brief 生成单个杂波块
     */
    void generateClutterBlock(ClutterBlock& block, const ClutterParams& params,
                               const SpectrumParams& spectrumParams);

    /**
     * @brief 生成唯一索引
     */
    size_t generateUniqueIndex();

private:
    std::vector<ClutterBlock> m_blocks;
    std::vector<ComplexVector> m_dataPool;  // 实际数据存储

    size_t m_poolSize;
    size_t m_sequenceLength;

    std::mutex m_mutex;
    std::atomic<size_t> m_accessIndex{0};

    bool m_initialized = false;
    ClutterParams m_cachedParams;
    SpectrumParams m_cachedSpectrumParams;
};

/**
 * @brief 杂波管理器（高层接口）
 * @details 管理杂波生成、内存池、以及回波调制
 */
class ClutterManager {
public:
    explicit ClutterManager();
    ~ClutterManager();

    /**
     * @brief 初始化杂波系统
     * @param params 杂波参数
     * @param spectrumParams 频谱参数
     * @param poolSize 内存池大小
     * @param sequenceLength 序列长度
     */
    void initialize(const ClutterParams& params, const SpectrumParams& spectrumParams,
                   size_t poolSize = 100, size_t sequenceLength = 1024);

    /**
     * @brief 为网格单元生成杂波序列
     * @param cell 网格单元
     * @param numPulses CPI 内脉冲数
     * @return 杂波序列矩阵 [numPulses x sequenceLength]
     */
    ComplexMatrix generateCellClutter(const ClutterCell& cell, int numPulses);

    /**
     * @brief 获取调制后的杂波（考虑天线增益）
     * @param cell 网格单元
     * @param pulseIndex 脉冲索引
     * @return 调制后的杂波序列
     */
    ComplexVector getModulatedClutter(const ClutterCell& cell, int pulseIndex);

    /**
     * @brief 保存杂波数据到文件
     */
    bool saveClutterData(const std::string& filename);

    /**
     * @brief 从文件加载杂波数据
     */
    bool loadClutterData(const std::string& filename);

    /**
     * @brief 获取内存池引用
     */
    ClutterMemoryPool& getMemoryPool() { return m_memoryPool; }

private:
    ClutterMemoryPool m_memoryPool;
    ClutterParams m_params;
    SpectrumParams m_spectrumParams;

    std::unique_ptr<ZMNLGenerator> m_zmnlGenerator;
    std::unique_ptr<SIRPGenerator> m_sirpGenerator;

    bool m_useSIRP = true;  // 默认使用 SIRP 方法
};
