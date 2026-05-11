#include "ChassisManager.hpp"
#include "main.h"
#include <algorithm>
#include "ChassisConfig.hpp"

namespace auv {
namespace control {

ChassisManager::ChassisManager() {
  applyConfig(DEFAULT_CHASSIS_CONFIG);
}

ChassisManager::ChassisManager(const ChassisConfig& cfg) {
  applyConfig(cfg);
}

void ChassisManager::applyConfig(const ChassisConfig& cfg) {
  config_ = cfg;
  for (int i = 0; i < 4; i++) {
    profiles_[i].setLimits(cfg.profile.default_max_v, cfg.profile.default_max_a);

    PID_Controller::Config pos_cfg;
    pos_cfg.kp = cfg.pos_pid.kp;
    pos_cfg.ki = cfg.pos_pid.ki;
    pos_cfg.kd = cfg.pos_pid.kd;
    pos_cfg.i_limit = cfg.pos_pid.i_limit;
    pos_cfg.output_limit = cfg.pos_pid.output_limit;
    pos_cfg.dt = cfg.pos_pid.dt;
    pos_pids_[i].setConfig(pos_cfg);

    PID_Controller::Config vel_cfg;
    vel_cfg.kp = cfg.vel_pid.kp;
    vel_cfg.ki = cfg.vel_pid.ki;
    vel_cfg.kd = cfg.vel_pid.kd;
    vel_cfg.i_limit = cfg.vel_pid.i_limit;
    vel_cfg.output_limit = cfg.vel_pid.output_limit;
    vel_cfg.dt = cfg.vel_pid.dt;
    vel_pids_[i].setConfig(vel_cfg);
  }
}

ControlLevel ChassisManager::getControlLevel() const {
  return level_;
}

PID_Controller::Config ChassisManager::getPIDConfig(int axis, bool is_pos_ring) const {
  if (axis < 0 || axis >= 4) return {};
  return is_pos_ring ? pos_pids_[axis].getConfig() : vel_pids_[axis].getConfig();
}

void ChassisManager::getProfileLimits(int axis, float& max_v, float& max_a) const {
  if (axis >= 0 && axis < 4) {
    max_v = profiles_[axis].getMaxV();
    max_a = profiles_[axis].getMaxA();
  }
}

void ChassisManager::configureProfile(int axis, float max_v, float max_a) {
  if (axis >= 0 && axis < 4) {
    float v = (max_v >= 0.0f) ? max_v : profiles_[axis].getMaxV();
    float a = (max_a >= 0.0f) ? max_a : profiles_[axis].getMaxA();
    profiles_[axis].setLimits(v, a);
  }
}

void ChassisManager::setActuatorForces(const float forces[4]) {
  for (int i = 0; i < 4; i++) target_forces_[i] = forces[i];
}

void ChassisManager::configurePID(int axis, bool is_pos_ring, float kp,
                                  float ki, float kd, float i_limit,
                                  float out_limit) {
  if (axis < 0 || axis >= 4)
    return;
  
  // 获取当前配置，用于增量修改
  PID_Controller::Config cfg = getPIDConfig(axis, is_pos_ring);
  
  if (kp >= 0.0f) cfg.kp = kp;
  if (ki >= 0.0f) cfg.ki = ki;
  if (kd >= 0.0f) cfg.kd = kd;
  if (i_limit >= 0.0f) cfg.i_limit = i_limit;
  if (out_limit >= 0.0f) cfg.output_limit = out_limit;
  
  cfg.dt = 0.01f; // 步长固定
  
  if (is_pos_ring)
    pos_pids_[axis].setConfig(cfg);
  else
    vel_pids_[axis].setConfig(cfg);
}

void ChassisManager::setControlLevel(ControlLevel new_level,
                                     const float actual_p[4],
                                     const float actual_v[4]) {
  if (new_level == level_)
    return;
  // 切换到 POSITION：需要将影子平滑器与真实传感器对齐，且对 Z 轴采用积分继承策略
  if (new_level == ControlLevel::POSITION) {
    for (int i = 0; i < 4; i++) {
      profiles_[i].align(actual_p[i], actual_v[i]);
      if (i == 2) {
        // 对 Z 轴采用无扰动继承：将上一周期输出作为积分初值（由 PID 内部限幅）
        vel_pids_[i].setIntegral(last_output_forces_[i]);
      } else {
        vel_pids_[i].reset();
      }
    }
  }

  // 切换到 VELOCITY：对齐影子状态并清除速度环积分
  else if (new_level == ControlLevel::VELOCITY) {
    for (int i = 0; i < 4; i++) {
      profiles_[i].align(actual_p[i], actual_v[i]);
      vel_pids_[i].reset();
    }
  }

  // 切换到 ACTUATOR：只改变层级，不清空 target_forces_
  else if (new_level == ControlLevel::ACTUATOR) {
    // 保持 target_forces_ 不变，使外部直接写入的力能立即生效
  }

  // 切换到 NONE（安全停机）：清空目标推力
  else if (new_level == ControlLevel::NONE) {
    target_forces_.fill(0.0f);
  }

  level_ = new_level;
}

std::array<float, 4> ChassisManager::update(const float actual_p[4],
                                            const float actual_v[4],
                                            const float target_p[4]) {
  std::array<float, 4> output_forces = {0};
  uint32_t now = HAL_GetTick();
  float dt = (last_update_tick_ == 0)
                 ? 0.01f
                 : (float)(now - last_update_tick_) / 1000.0f;
  last_update_tick_ = now;

  // 防止 dt 异常：限定在 [1ms, 100ms] 范围
  if (dt > 0.1f) dt = 0.1f;
  if (dt <= 0.0f) dt = 0.001f;

  if (level_ == ControlLevel::NONE)
    return output_forces;

  std::array<float, 4> v_target_world = {0};
  for (int i = 0; i < 4; i++) {
    ProfileState d = profiles_[i].update(target_p[i], dt);
    if (level_ == ControlLevel::POSITION) {
      // 位置环的导数项使用速度误差 (v_ref - v_actual)
      float pos_derivative = d.v - actual_v[i];
      v_target_world[i] = pos_pids_[i].compute(d.p - actual_p[i], dt, pos_derivative) + d.v;
    } else {
      v_target_world[i] = d.v;
    }
  }

  // 坐标系转换：将世界系 (NED) 的目标速度转换为机体系 (Body)
  float v_target_body[4];
  CoordinateManager::worldToBody(actual_p[3], v_target_world[0], v_target_world[1], 
                                  v_target_body[0], v_target_body[1]);
  v_target_body[2] = v_target_world[2]; // Z 轴相同
  v_target_body[3] = v_target_world[3]; // Yaw 轴相同

  for (int i = 0; i < 4; i++) {
    float f_base = 0.0f;
    if (level_ == ControlLevel::POSITION || level_ == ControlLevel::VELOCITY) {
      // 使用机体系下的目标速度与真实速度进行闭环
      // 速度环导数项使用 (a_ref - a_actual)，其中 a_actual 由实际速度差分得到
      float a_ref = profiles_[i].getState().a;
      float a_actual = 0.0f;
      if (last_update_tick_ != 0 && dt > 0.0f) {
        a_actual = (actual_v[i] - last_actual_v_[i]) / dt;
      }
      float vel_derivative = a_ref - a_actual;

      f_base = vel_pids_[i].compute(v_target_body[i] - actual_v[i], dt, vel_derivative);

      // 前馈加速度仍然叠加
      f_base += 1.0f * a_ref;
    }
    output_forces[i] = f_base + target_forces_[i];
  }

  last_z_thrust_ = output_forces[2];
  last_output_forces_ = output_forces; // 更新快照

  // 更新实际速度快照，用于下一周期的加速度估计
  for (int i = 0; i < 4; i++) last_actual_v_[i] = actual_v[i];
  return output_forces;
}

} // namespace control
} // namespace auv
