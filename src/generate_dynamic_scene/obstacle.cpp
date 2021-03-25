#include "generate_dynamic_scene/obstacle.hpp"

namespace hdi_plan {

Obstacle::Obstacle(std::string name, Obstacle_type type, bool operation, float size, Eigen::Vector3d position)
	: name_(name), type_(type), operation_(operation), size_(size), position_(position) {

}

Obstacle::~Obstacle() = default;
}