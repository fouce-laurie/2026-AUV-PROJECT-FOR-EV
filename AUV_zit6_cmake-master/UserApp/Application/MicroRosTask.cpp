#include "AppMain.hpp"
#include "MicroRosTask.hpp"
#include "GlobalContext.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rcutils/allocator.h>
#include <rmw_microros/rmw_microros.h>
#include "SoftWatchdog.hpp"

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/u_int8.h>
#include <zit6_interfaces/msg/zit_setpoint.h>
#include <zit6_interfaces/msg/zit_status.h>
#include <zit6_interfaces/msg/zit_pid.h>
#include <zit6_interfaces/msg/zit_pid_status.h>

extern "C" {
bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *errcode);
size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode);
void *microros_allocate(size_t size, void *state);
void microros_deallocate(void *ptr, void *state);
void *microros_reallocate(void *ptr, size_t new_size, void *state);
void *microros_zero_allocate(size_t number_of_elements, size_t size_t_of_element, void *state);
}

// 实例指针定义
MicroRosTask *MicroRosTask::instance_ = nullptr;

// --- 成员回调实现 ---
void MicroRosTask::onZitPid(const void *msgin) {
    const auto *msg = (const zit6_interfaces__msg__ZitPid *)msgin;
    if (!std::isfinite(msg->kp) || !std::isfinite(msg->ki) || !std::isfinite(msg->kd)) return;
    auv::control::chassis.configurePID(msg->axis, msg->is_pos_ring, msg->kp, msg->ki, msg->kd, msg->i_limit, msg->out_limit);
    if (msg->is_pos_ring) auv::control::chassis.configureProfile(msg->axis, msg->max_v, msg->max_a);
}

void MicroRosTask::onZitSetpoint(const void *msgin) {
    const auto *msg = (const zit6_interfaces__msg__ZitSetpoint *)msgin;
    last_received_seq = msg->seq;
    if (!std::isfinite(msg->x) || !std::isfinite(msg->y) || !std::isfinite(msg->z) || !std::isfinite(msg->yaw)) return;
    if (!is_system_armed) return;

    uint32_t level = msg->control_key & 0x03;
    bool is_body = (msg->control_key & 0x10) != 0;
    bool is_inc = (msg->control_key & 0x20) != 0;
    uint32_t mask = msg->type_mask;
    float val[4] = {msg->x, msg->y, msg->z, msg->yaw};

    auto nav = auv::shared::snapshotNavState();
    if ((level == 0 || level == 1) && !auv::shared::isNavigationValid(nav)) return;

    if (level == 2) { // FORCE
        float fx = val[0], fy = val[1];
        if (!is_body) auv::control::CoordinateManager::worldToBody(nav.yaw, val[0], val[1], fx, fy);
        float forces[4] = {fx, fy, val[2], val[3]};
        taskENTER_CRITICAL();
        auv::control::chassis.setActuatorForces(forces);
        float actual_p[4] = {nav.x, nav.y, nav.z, nav.yaw};
        float actual_v[4] = {nav.vx, nav.vy, nav.vz, nav.vyaw};
        auv::control::chassis.setControlLevel(auv::ControlLevel::ACTUATOR, actual_p, actual_v);
        taskEXIT_CRITICAL();
    } else if (level == 1) { // VEL
        taskENTER_CRITICAL();
        for (int i = 0; i < 4; i++) target_p[i] = val[i];
        float actual_p[4] = {nav.x, nav.y, nav.z, nav.yaw};
        float actual_v[4] = {nav.vx, nav.vy, nav.vz, nav.vyaw};
        auv::control::chassis.setControlLevel(auv::ControlLevel::VELOCITY, actual_p, actual_v);
        taskEXIT_CRITICAL();
    } else if (level == 0) { // POS
        taskENTER_CRITICAL();
        if (is_body && is_inc) {
            float wx, wy;
            auv::control::CoordinateManager::bodyToWorld(nav.yaw, val[0], val[1], wx, wy);
            if (mask & 0x01) target_p[0] = nav.x + wx;
            if (mask & 0x02) target_p[1] = nav.y + wy;
            if (mask & 0x04) target_p[2] = nav.z + val[2];
            if (mask & 0x08) target_p[3] = nav.yaw + val[3];
        } else {
            for (int i = 0; i < 4; i++) if (mask & (1 << i)) target_p[i] = val[i];
        }
        float actual_p[4] = {nav.x, nav.y, nav.z, nav.yaw};
        float actual_v[4] = {nav.vx, nav.vy, nav.vz, nav.vyaw};
        auv::control::chassis.setControlLevel(auv::ControlLevel::POSITION, actual_p, actual_v);
        taskEXIT_CRITICAL();
    }
}

void MicroRosTask::onArmHeartbeat(const void *msgin) {
    const auto *msg = (const std_msgs__msg__UInt32 *)msgin;
    taskENTER_CRITICAL();
    last_arm_heartbeat_ms = HAL_GetTick();
    last_arm_heartbeat_data = msg->data;
    if (!is_system_armed) {
        if (arm_heartbeat_count == 0) arm_start_ms = last_arm_heartbeat_ms;
        arm_heartbeat_count++;
    }
    taskEXIT_CRITICAL();
}

void MicroRosTask::onInsCommand(const void *msgin) {
    const auto *message = static_cast<const std_msgs__msg__UInt8 *>(msgin);
    if (message == nullptr) return;
    switch (message->data) {
        case 1: auv::device::ins_driver.setDvlPower(true); break;
        case 2: auv::device::ins_driver.setDvlPower(false); break;
        case 3: auv::device::ins_driver.restart(); break;
        case 4: auv::device::ins_driver.resetPosition(); break;
        case 5: auv::device::ins_driver.setInitialPosition(30.0, 120.0); break;
    }
}

void MicroRosTask::onServoCmd(const void *msgin) {
    const auto *msg = (const std_msgs__msg__Float32 *)msgin;
    auv::device::motor_driver.setServoAngle(msg->data);
}

void MicroRosTask::onLedCmd(const void *msgin) {
    const auto *msg = (const std_msgs__msg__UInt8 *)msgin;
    auv::device::motor_driver.setLightState(msg->data);
}

void MicroRosTask::cleanupMicroRos() {
    rclc_executor_fini(&executor_);
    rcl_publisher_fini(&pos_pub_, &node_);
    rcl_publisher_fini(&vel_pub_, &node_);
    rcl_publisher_fini(&thr_pub_, &node_);
    rcl_publisher_fini(&zithbt_pub_, &node_);
    rcl_publisher_fini(&status_pub_, &node_);
    rcl_publisher_fini(&pid_status_pub_, &node_);
    rcl_subscription_fini(&setpoint_sub_, &node_);
    rcl_subscription_fini(&arm_sub_, &node_);
    rcl_subscription_fini(&ins_cmd_sub_, &node_);
    rcl_subscription_fini(&pid_sub_, &node_);
    rcl_subscription_fini(&servo_sub_, &node_);
    rcl_subscription_fini(&led_sub_, &node_);
    rcl_node_fini(&node_);
    rclc_support_fini(&support_);
    memset(&support_, 0, sizeof(support_));
    memset(&node_, 0, sizeof(node_));
    memset(&executor_, 0, sizeof(executor_));
}

void MicroRosTask::run() {
    MicroRosTask::instance_ = this;
    rmw_uros_set_custom_transport(true, (void *)&huart2, cubemx_transport_open, cubemx_transport_close, cubemx_transport_write, cubemx_transport_read);
    rcutils_allocator_t allocator = rcutils_get_zero_initialized_allocator();
    allocator.allocate = microros_allocate; allocator.deallocate = microros_deallocate;
    allocator.reallocate = microros_reallocate; allocator.zero_allocate = microros_zero_allocate;
    rcutils_set_default_allocator(&allocator);
    rcl_allocator_t rcl_allocator = rcl_get_default_allocator();

    uint32_t last_hbt_pub_tick = 0, last_vel_pub_tick = 0, last_thr_pub_tick = 0, last_pos_pub_tick = 0, last_status_pub_tick = 0;
    enum uros_state { WAITING_AGENT, AGENT_CONNECTED } state = WAITING_AGENT;

    for (;;) {
        uint32_t now_ms = HAL_GetTick();
        if (state == WAITING_AGENT) {
            if (RCL_RET_OK == rmw_uros_ping_agent(200, 1)) {
                if (RCL_RET_OK == rclc_support_init(&support_, 0, NULL, &rcl_allocator)) {
                    rmw_uros_sync_session(100);
                    rclc_node_init_default(&node_, "zit6_node", "", &support_);
                    std_msgs__msg__Float32MultiArray__init(&pos_fb_msg_); pos_fb_msg_.data.data = pos_buf_; pos_fb_msg_.data.size = 4; pos_fb_msg_.data.capacity = 4;
                    std_msgs__msg__Float32MultiArray__init(&vel_fb_msg_); vel_fb_msg_.data.data = vel_buf_; vel_fb_msg_.data.size = 4; vel_fb_msg_.data.capacity = 4;
                    std_msgs__msg__Float32MultiArray__init(&thr_fb_msg_); thr_fb_msg_.data.data = thr_buf_; thr_fb_msg_.data.size = 4; thr_fb_msg_.data.capacity = 4;
                    std_msgs__msg__UInt32__init(&node_heartbeat_msg_);
                    std_msgs__msg__UInt32__init(&arm_msg_);
                    std_msgs__msg__UInt8__init(&ins_cmd_msg_);
                    std_msgs__msg__UInt8__init(&led_msg_);
                    std_msgs__msg__Float32__init(&servo_msg_);
                    zit6_interfaces__msg__ZitStatus__init(&status_msg_);

                    rclc_publisher_init_default(&pos_pub_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/zit6/state/pos");
                    rclc_publisher_init_default(&vel_pub_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/zit6/state/vel");
                    rclc_publisher_init_default(&thr_pub_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/zit6/state/thr");
                    rclc_publisher_init_default(&zithbt_pub_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32), "/zit6/state/zithbt");
                    rclc_publisher_init_default(&status_pub_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(zit6_interfaces, msg, ZitStatus), "/zit6/state/status");
                    rclc_publisher_init_default(&pid_status_pub_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(zit6_interfaces, msg, ZitPidStatus), "/zit6/state/pid_status");

                    rclc_subscription_init_default(&led_sub_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "/zit6/cmd/light");
                    rclc_subscription_init_default(&servo_sub_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/zit6/cmd/servo");
                    rclc_subscription_init_default(&setpoint_sub_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(zit6_interfaces, msg, ZitSetpoint), "/zit6/cmd/setpoint");
                    rclc_subscription_init_default(&arm_sub_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32), "/zit6/cmd/agxhbt");//0  1 ins   
                    rclc_subscription_init_default(&ins_cmd_sub_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "/zit6/cmd/ins");//1开关dvl 2重启惯导 3重置位置 4设置初始位置  
                    rclc_subscription_init_default(&pid_sub_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(zit6_interfaces, msg, ZitPid), "/zit6/cmd/pid");

                    rclc_executor_init(&executor_, &support_.context, 12, &rcl_allocator);
                    rclc_executor_add_subscription(&executor_, &setpoint_sub_, &setpoint_msg_, &MicroRosTask::setpointCb, ON_NEW_DATA);
                    rclc_executor_add_subscription(&executor_, &arm_sub_, &arm_msg_, &MicroRosTask::armCb, ON_NEW_DATA);
                    rclc_executor_add_subscription(&executor_, &ins_cmd_sub_, &ins_cmd_msg_, &MicroRosTask::insCmdCb, ON_NEW_DATA);
                    rclc_executor_add_subscription(&executor_, &pid_sub_, &pid_msg_, &MicroRosTask::zitPidCb, ON_NEW_DATA);
                    rclc_executor_add_subscription(&executor_, &servo_sub_, &servo_msg_, &MicroRosTask::servoCb, ON_NEW_DATA);
                    rclc_executor_add_subscription(&executor_, &led_sub_, &led_msg_, &MicroRosTask::ledCb, ON_NEW_DATA);
                    state = AGENT_CONNECTED;
                }
            } else vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            static uint32_t last_pid_pub_tick = 0;
            // Robust ping: 500ms timeout, 3 failures
            if (RCL_RET_OK != rmw_uros_ping_agent(500, 3)) {
                cleanupMicroRos(); state = WAITING_AGENT;
            } else {
                auv::device::SoftWatchdog::getInstance().feed(auv::device::SoftWatchdog::Component::MICROROS);
                rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(1));
                if (now_ms - last_hbt_pub_tick >= 1000) { last_hbt_pub_tick = now_ms; node_heartbeat_msg_.data = now_ms; rcl_publish(&zithbt_pub_, &node_heartbeat_msg_, NULL); }
                if (now_ms - last_vel_pub_tick >= 20) { last_vel_pub_tick = now_ms; auto nav = auv::shared::snapshotNavState(); vel_buf_[0] = nav.vx; vel_buf_[1] = nav.vy; vel_buf_[2] = nav.vz; vel_buf_[3] = nav.vyaw; rcl_publish(&vel_pub_, &vel_fb_msg_, NULL); }
                if (now_ms - last_thr_pub_tick >= 33) { last_thr_pub_tick = now_ms; taskENTER_CRITICAL(); for (int i = 0; i < 4; i++) thr_buf_[i] = last_output_forces[i]; taskEXIT_CRITICAL(); rcl_publish(&thr_pub_, &thr_fb_msg_, NULL); }
                if (now_ms - last_pos_pub_tick >= 33) { last_pos_pub_tick = now_ms; auto nav = auv::shared::snapshotNavState(); pos_buf_[0] = nav.x; pos_buf_[1] = nav.y; pos_buf_[2] = nav.z; pos_buf_[3] = nav.yaw; rcl_publish(&pos_pub_, &pos_fb_msg_, NULL); }
                if (now_ms - last_status_pub_tick >= 100) {
                    last_status_pub_tick = now_ms; auv::NavState nav = auv::shared::snapshotNavState();
                    taskENTER_CRITICAL();
                    status_msg_.is_armed = is_system_armed; status_msg_.arm_mode = (uint8_t)last_arm_heartbeat_data; status_msg_.control_level = (uint8_t)auv::control::chassis.getControlLevel();
                    status_msg_.ins_state = nav.imu_state; status_msg_.navigation_ready = auv::shared::isNavigationValid(nav); for (int i = 0; i < 4; i++) status_msg_.forces[i] = last_output_forces[i];
                    status_msg_.cycle_time_ms = (float)last_dt_ms; status_msg_.battery_voltage = 0.0f; status_msg_.error_flags = 0;
                    taskEXIT_CRITICAL();
                    rcl_publish(&status_pub_, &status_msg_, NULL);
                }
                if (now_ms - last_pid_pub_tick >= 1000) {
                    last_pid_pub_tick = now_ms;
                    for (int i = 0; i < 4; i++) {
                        auto p_cfg = auv::control::chassis.getPIDConfig(i, true);
                        pid_status_msg_.pos_kp[i] = p_cfg.kp; auv::control::chassis.getProfileLimits(i, pid_status_msg_.pos_max_v[i], pid_status_msg_.pos_max_a[i]); pid_status_msg_.pos_out_limit[i] = p_cfg.output_limit;
                        auto v_cfg = auv::control::chassis.getPIDConfig(i, false);
                        pid_status_msg_.vel_kp[i] = v_cfg.kp; pid_status_msg_.vel_ki[i] = v_cfg.ki; pid_status_msg_.vel_kd[i] = v_cfg.kd; pid_status_msg_.vel_i_limit[i] = v_cfg.i_limit; pid_status_msg_.vel_out_limit[i] = v_cfg.output_limit;
                    }
                    rcl_publish(&pid_status_pub_, &pid_status_msg_, NULL);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void UserApp_MicroRosTask(void *argument) {
    (void)argument;
    MicroRosTask runner;
    runner.run();
}
