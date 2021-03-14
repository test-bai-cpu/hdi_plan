#include "generate_dynamic_scene/obstacle.hpp"

namespace hdi_plan {

Obstacle::Obstacle(std::string name, Obstacle_type type, bool operation, Eigen::Vector3d position)
	: name_(name), type_(type), operation_(operation), position_(position) {

}

Obstacle::~Obstacle() {
}

}