#pragma once

#include "PID_Controller.hpp"
#include "KinematicProfile.hpp"
#include "CoordinateManager.hpp"
#include "ChassisConfig.hpp"
#include "CommonConfig.hpp"
#include <array>

namespace auv {
namespace control {

/**
 * @class ChassisManager
 * @brief 整合了平滑、计算与状态切换的底盘大脑
 */
class ChassisManager {
public:
    ChassisManager();
    ChassisManager(const ChassisConfig& cfg);

    /**
     * @brief 应用外部配置（可用于运行时切换参数）
     */
    void applyConfig(const ChassisConfig& cfg);

    /**
     * @brief 获取当前控制层级
     */
    ControlLevel getControlLevel() const;

    /**
     * @brief 执行 100Hz 级联控制演进 (自动计算 dt)
     * @param actual_p 当前位姿 (NED)
     * @param actual_v 当前速度 (Body)
     * @param target_p 目标位姿 (NED)
     * @return std::array<float, 4> 计算出的 4-DOF 归一化力矢量
     */
    std::array<float, 4> update(const float actual_p[4], const float actual_v[4], const float target_p[4]);

    /**
     * @brief 配置指定轴的 PID 参数
     * @param axis 轴索引 (0-3: X, Y, Z, Yaw)
     * @param is_pos_ring 是否为位置环
     * @param kp, ki, kd, i_limit, out_limit 参数
     */
    void configurePID(int axis, bool is_pos_ring, float kp, float ki, float kd, float i_limit, float out_limit);
    
    /**
     * @brief 获取指定轴的 PID 配置
     */
    PID_Controller::Config getPIDConfig(int axis, bool is_pos_ring) const;

    /**
     * @brief 获取指定轴的运动学约束
     */
    void getProfileLimits(int axis, float& max_v, float& max_a) const;

    /**
     * @brief 配置指定轴的运动学约束
     * @param axis 轴索引 (0-3: X, Y, Z, Yaw)
     * @param max_v 最大速度 (若 < 0 则保留当前值)
     * @param max_a 最大加速度 (若 < 0 则保留当前值)
     */
    void configureProfile(int axis, float max_v, float max_a);

    /**
     * @brief 切换控制层级 (Bumpless Transfer)
     * @param new_level 目标控制层级
     * @param actual_p 当前真实位置（用于对齐）
     * @param actual_v 当前真实速度（用于对齐）
     */
    void setControlLevel(ControlLevel new_level, const float actual_p[4], const float actual_v[4]);

    /**
     * @brief 直接设置执行器输出力 (作为 ACTUATOR 输出或闭环 Bias)
     * @param forces 归一化力矢量 [Fx, Fy, Fz, Mz]
     */
    void setActuatorForces(const float forces[4]);

private:
    ControlLevel level_ = ControlLevel::NONE;
    
    std::array<KinematicProfile, 4> profiles_; ///< 4轴影子平滑器矩阵
    std::array<PID_Controller, 4> pos_pids_;  ///< 位置环 (P控制)
    std::array<PID_Controller, 4> vel_pids_;  ///< 速度环 (PI控制)
    
    std::array<float, 4> target_forces_  = {0}; ///< 压入推力环的直接力/偏置
    std::array<float, 4> last_output_forces_ = {0}; ///< 记录上周期的最终输出，用于无扰切换
    float last_z_thrust_ = 0.0f;               ///< 记录上周期的 Z 轴推力，用于 Trim Pre-loading
    std::array<float, 4> last_actual_v_ = {0}; ///< 上周期实际速度，用于计算加速度（用于 D 项）
    uint32_t last_update_tick_ = 0;            ///< 用于自动计算 dt
    ChassisConfig config_ = DEFAULT_CHASSIS_CONFIG; ///< 当前应用的底盘参数
};

} // namespace control
} // namespace auv
