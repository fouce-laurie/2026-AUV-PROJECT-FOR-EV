#ifndef __COORDINATE_MANAGER_HPP
#define __COORDINATE_MANAGER_HPP

#include "CommonConfig.hpp"
#include <math.h>

namespace auv {

class CoordinateManager {
public:
    CoordinateManager() {
        reset();
    }

    void updateBasePose(const NavState& state) {
        current_base_ = state;
    }

    void body2World(float body_x, float body_y, float body_z, float& world_x, float& world_y, float& world_z) {
        float cos_y = cosf(current_base_.yaw * Constants::DEG2RAD);
        float sin_y = sinf(current_base_.yaw * Constants::DEG2RAD);
        
        world_x = body_x * cos_y - body_y * sin_y;
        world_y = body_x * sin_y + body_y * cos_y;
        world_z = body_z;
    }

    void world2Body(float world_x, float world_y, float world_z, float& body_x, float& body_y, float& body_z) {
        float cos_y = cosf(-current_base_.yaw * Constants::DEG2RAD);
        float sin_y = sinf(-current_base_.yaw * Constants::DEG2RAD);
        
        float dx = world_x - current_base_.x;
        float dy = world_y - current_base_.y;
        
        body_x = dx * cos_y - dy * sin_y;
        body_y = dx * sin_y + dy * cos_y;
        body_z = world_z - current_base_.z;
    }

private:
    NavState current_base_;
    void reset() {
        memset(&current_base_, 0, sizeof(NavState));
    }
};

} // namespace auv

#endif
