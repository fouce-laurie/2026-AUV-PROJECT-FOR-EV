#include "AppMain.hpp"
#include "ControlTask.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SoftWatchdog.hpp"
#include <string.h>

using namespace auv::device;
using namespace auv::control;


void ControlTask::fillActualState(const auv::NavState &nav, float (&actual_p)[4], float (&actual_v)[4]) {
    actual_p[0] = nav.x;
    actual_p[1] = nav.y;
    actual_p[2] = nav.z;
    actual_p[3] = nav.yaw;

    actual_v[0] = nav.vx;
    actual_v[1] = nav.vy;
    actual_v[2] = nav.vz;
    actual_v[3] = nav.vyaw;
}

void ControlTask::run() {
    init();

    for (;;) {
        refreshHardwareWatchdogIfNeeded();

        const uint32_t now = HAL_GetTick();
        last_dt_ms = static_cast<float>(now - last_tick_);
        last_tick_ = now;

        auv::NavState nav = updateNavigation();
        handleArmState(nav, now);
        computeAndPublish(nav);

        vTaskDelayUntil(&last_wake_time_, pdMS_TO_TICKS(kLoopPeriodMs));
    }
}

void ControlTask::init() {
    memset(ins_rx_buffer, 0, sizeof(ins_rx_buffer));
    memset(&motor_tx_packet, 0, sizeof(motor_tx_packet));

    ins_driver.init();
    SoftWatchdog::getInstance().init();

    last_wake_time_ = xTaskGetTickCount();
    last_tick_ = HAL_GetTick();
}

void ControlTask::refreshHardwareWatchdogIfNeeded() {
    if (SoftWatchdog::getInstance().check()) {
        HAL_IWDG_Refresh(&hiwdg1);
    }
}

auv::NavState ControlTask::updateNavigation() {
    auv::NavState nav = auv::shared::snapshotNavState();
    ins_driver.update(nav);

    // 保护读取全局深度快照，避免 IICTask 并发写导致短时 0 值
    float depth_snapshot = 0.0f;
    taskENTER_CRITICAL();
    depth_snapshot = current_depth_z;
    taskEXIT_CRITICAL();

    nav.z = depth_snapshot;

    taskENTER_CRITICAL();
    shared_nav_state = nav;
    taskEXIT_CRITICAL();

    return nav;
}

void ControlTask::setControlLevelNone(const auv::NavState &nav) {
    float actual_p[4];
    float actual_v[4];
    fillActualState(nav, actual_p, actual_v);
    chassis.setControlLevel(auv::ControlLevel::NONE, actual_p, actual_v);
}

void ControlTask::forceDisarmWithNeutralLevel(const auv::NavState &nav) {
    taskENTER_CRITICAL();
    is_system_armed = false;
    arm_heartbeat_count = 0;
    taskEXIT_CRITICAL();
    setControlLevelNone(nav);
}

void ControlTask::handleArmState(const auv::NavState &nav, uint32_t now) {
    taskENTER_CRITICAL();
    const bool armed_snapshot = is_system_armed;
    const uint32_t heartbeat_snapshot = last_arm_heartbeat_ms;
    const uint32_t heartbeat_count_snapshot = arm_heartbeat_count;
    const uint32_t arm_start_snapshot = arm_start_ms;
    taskEXIT_CRITICAL();

    if (armed_snapshot) {
        if (now - heartbeat_snapshot > kArmedHeartbeatTimeoutMs) {
            forceDisarmWithNeutralLevel(nav);
        }
        return;
    }

    if (chassis.getControlLevel() != auv::ControlLevel::NONE) {
        setControlLevelNone(nav);
    }

    if (heartbeat_count_snapshot >= kArmMinHeartbeatCount &&
        (now - arm_start_snapshot >= kArmMinDurationMs)) {
        taskENTER_CRITICAL();
        const uint32_t hbt_data = last_arm_heartbeat_data;
        taskEXIT_CRITICAL();

        if (hbt_data == kRemoteModeHeartbeatData || auv::shared::isNavigationValid(nav)) {
            taskENTER_CRITICAL();
            is_system_armed = true;
            taskEXIT_CRITICAL();
        } else {
            taskENTER_CRITICAL();
            arm_heartbeat_count = 0;
            taskEXIT_CRITICAL();
        }
    }

    if (now - heartbeat_snapshot > kDisarmedHeartbeatTimeoutMs) {
        taskENTER_CRITICAL();
        arm_heartbeat_count = 0;
        taskEXIT_CRITICAL();
    }
}

void ControlTask::computeAndPublish(const auv::NavState &nav) {
    float actual_p[4];
    float actual_v[4];
    fillActualState(nav, actual_p, actual_v);

    float target_snapshot[4];
    taskENTER_CRITICAL();
    for (int i = 0; i < 4; ++i) {
        target_snapshot[i] = target_p[i];
    }
    taskEXIT_CRITICAL();

    auto forces = chassis.update(actual_p, actual_v, target_snapshot);

    taskENTER_CRITICAL();
    for (int i = 0; i < 4; ++i) {
        last_output_forces[i] = forces[i];
    }
    const bool armed = is_system_armed;
    taskEXIT_CRITICAL();

    if (armed) {
        motor_driver.publishThrust(forces[0], forces[1], forces[2], forces[3]);
    } else {
        motor_driver.publishThrust(0, 0, 0, 0);
    }
}

void UserApp_ControlTask(void *argument) {
    (void)argument;
    ControlTask runner;
    runner.run();
}
