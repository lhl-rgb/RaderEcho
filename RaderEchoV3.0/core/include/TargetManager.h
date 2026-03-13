#pragma once

#include "Types.h"
#include <vector>
#include <memory>
#include <functional>

/**
 * @brief 目标运动模型基类
 */
class MotionModelBase {
public:
    virtual ~MotionModelBase() = default;

    /**
     * @brief 更新目标位置
     * @param target 目标参数（引用，直接修改）
     * @param dt 时间增量
     */
    virtual void updatePosition(TargetParams& target, SignalType dt) = 0;

    /**
     * @brief 获取模型类型
     */
    virtual MotionModel getModelType() const = 0;
};

/**
 * @brief 静止模型
 */
class StationaryModel : public MotionModelBase {
public:
    void updatePosition(TargetParams& target, SignalType dt) override;
    MotionModel getModelType() const override { return MotionModel::Stationary; }
};

/**
 * @brief 匀速运动模型
 */
class ConstantVelocityModel : public MotionModelBase {
public:
    void updatePosition(TargetParams& target, SignalType dt) override;
    MotionModel getModelType() const override { return MotionModel::ConstantVelocity; }
};

/**
 * @brief 匀加速运动模型
 */
class ConstantAccelerationModel : public MotionModelBase {
public:
    void updatePosition(TargetParams& target, SignalType dt) override;
    MotionModel getModelType() const override { return MotionModel::ConstantAcceleration; }
};

/**
 * @brief 变加速运动模型
 * @details 加速度随时间变化，支持加速度输入
 */
class VariableAccelerationModel : public MotionModelBase {
public:
    explicit VariableAccelerationModel(
        std::function<Acceleration3D(SignalType t)> accelFunc);
    void updatePosition(TargetParams& target, SignalType dt) override;
    MotionModel getModelType() const override { return MotionModel::VariableAcceleration; }

private:
    std::function<Acceleration3D(SignalType t)> m_accelFunc;
};

/**
 * @brief 圆周运动模型
 */
class CircularModel : public MotionModelBase {
public:
    void updatePosition(TargetParams& target, SignalType dt) override;
    MotionModel getModelType() const override { return MotionModel::Circular; }
};

/**
 * @brief 正弦机动模型
 */
class SineWaveModel : public MotionModelBase {
public:
    void updatePosition(TargetParams& target, SignalType dt) override;
    MotionModel getModelType() const override { return MotionModel::SineWave; }
};

/**
 * @brief 目标管理器
 * @details 管理多个目标的添加、删除、更新和状态获取
 */
class TargetManager {
public:
    TargetManager();
    ~TargetManager();

    /**
     * @brief 添加目标
     * @param target 目标参数
     * @return 目标 ID
     */
    size_t addTarget(const TargetParams& target);

    /**
     * @brief 删除目标
     * @param id 目标 ID
     */
    void removeTarget(size_t id);

    /**
     * @brief 更新目标参数
     * @param id 目标 ID
     * @param target 新的目标参数
     */
    void updateTarget(size_t id, const TargetParams& target);

    /**
     * @brief 获取目标
     * @param id 目标 ID
     * @return 目标参数指针（不存在则返回 nullptr）
     */
    TargetParams* getTarget(size_t id);

    /**
     * @brief 获取所有目标
     * @return 目标列表
     */
    std::vector<TargetParams> getAllTargets() const;

    /**
     * @brief 更新所有目标状态
     * @param dt 时间增量
     * @param currentTime 当前时间
     */
    void updateAllTargets(SignalType dt, SignalType currentTime);

    /**
     * @brief 更新目标 RCS（根据 Swerling 模型）
     * @param target 目标参数（引用）
     * @param cpiStart 是否为新 CPI 开始
     */
    void updateTargetRCS(TargetParams& target, bool cpiStart);

    /**
     * @brief 为目标设置运动模型
     * @param id 目标 ID
     * @param model 运动模型
     */
    void setMotionModel(size_t id, MotionModel model);

    /**
     * @brief 清除所有目标
     */
    void clear();

    /**
     * @brief 获取目标数量
     */
    size_t getTargetCount() const { return m_targets.size(); }

private:
    std::vector<TargetParams> m_targets;
    std::unordered_map<MotionModel, std::unique_ptr<MotionModelBase>> m_models;
    size_t m_nextId = 1;
};
