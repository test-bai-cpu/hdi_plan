#pragma once

#include <Eigen/Dense>

#include "utils/types.hpp"

namespace hdi_plan {

class Obstacle {
public:
	Obstacle(std::string name, Obstacle_type type, bool operation, Eigen::Vector3d position);
	~Obstacle();
	double get_operation() const {
		return this->operation_;
	};

private:
	std::string name_;
	Obstacle_type type_;
	bool operation_;
	Eigen::Vector3d position_;
};


