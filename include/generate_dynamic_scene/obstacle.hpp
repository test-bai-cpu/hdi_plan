#pragma once

#include <Eigen/Dense>

#include "utils/types.hpp"

namespace hdi_plan {

class Obstacle {
public:
	Obstacle(std::string name, Obstacle_type type, bool operation, double size, Eigen::Vector3d position);

	~Obstacle();

	std::string get_name() const {
		return this->name_;
	};

	Obstacle_type get_type() const {
		return this->type_;
	};

	bool get_operation() const {
		return this->operation_;
	};

	double get_size() const {
		return this->size_;
	};

	Eigen::Vector3d get_position() const {
		return this->position_;
	};
private:
	std::string name_;
	Obstacle_type type_;
	bool operation_;
	double size_;
	Eigen::Vector3d position_;
};


}