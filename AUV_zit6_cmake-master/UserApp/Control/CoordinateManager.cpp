#include "CoordinateManager.hpp"
#include <cmath>

namespace auv {
namespace control {

void CoordinateManager::worldToBody(float yaw, float world_x, float world_y, float& body_x, float& body_y) {
    float cos_y = std::cos(yaw);
    float sin_y = std::sin(yaw);
    
    body_x = world_x * cos_y + world_y * sin_y;
    body_y = -world_x * sin_y + world_y * cos_y;
}

void CoordinateManager::bodyToWorld(float yaw, float body_x, float body_y, float& world_x, float& world_y) {
    float cos_y = std::cos(yaw);
    float sin_y = std::sin(yaw);
    
    world_x = body_x * cos_y - body_y * sin_y;
    world_y = body_x * sin_y + body_y * cos_y;
}

} // namespace control
} // namespace auv
