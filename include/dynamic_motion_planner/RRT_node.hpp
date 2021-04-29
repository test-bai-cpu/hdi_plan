#pragma once

#include <Eigen/Dense>
#include <memory>
#include <limits>
#include <ompl/datastructures/BinaryHeap.h>

namespace hdi_plan {

class RRTNode {
public:
    RRTNode() = default;
	RRTNode(Eigen::Vector3d state);
    RRTNode(Eigen::Vector3d state, double time_from_start);
    ~RRTNode();

    // for nearest neighbor tree
    std::shared_ptr<RRTNode> parent;
    std::vector<std::shared_ptr<RRTNode>> children;

    // for extend part
    std::vector<std::pair<std::shared_ptr<RRTNode>, bool>> nbh;

    // for neighbors, original in is N0-(v), out is N0+(v), running in is Nr-(v), out is Nr+(v)
    std::vector<std::shared_ptr<RRTNode>> n0_in; // -
    std::vector<std::shared_ptr<RRTNode>> n0_out; // +
    std::vector<std::shared_ptr<RRTNode>> nr_in; // -
    std::vector<std::shared_ptr<RRTNode>> nr_out; // +

    struct node_compare{
        bool operator()(const std::shared_ptr<RRTNode>& a, const std::shared_ptr<RRTNode>& b){
            double a_min_cost = std::min(a->get_lmc(), a->get_g_cost());
            double b_min_cost = std::min(b->get_lmc(), b->get_g_cost());

            if (a_min_cost == b_min_cost) return a->get_g_cost() > b->get_g_cost();
            return a_min_cost > b_min_cost;
        }
    };

    // for check if the node is in the priority queue
    ompl::BinaryHeap<std::shared_ptr<RRTNode>, node_compare>::Element *handle;

    double get_lmc() const {
        return this->lmc_;
    };

    void set_lmc(double lmc) {
        this->lmc_ = lmc;
    };

    double get_g_cost() const {
        return this->g_cost_;
    };

    void set_g_cost(double g_cost) {
        this->g_cost_ = g_cost;
    };

	double get_time() const {
		return this->time_;
	};

	void set_time(double time_from_start) {
		this->time_ = time_from_start;
	};


	void set_state_by_vector(Eigen::Vector3d new_state) {
        this->state_ = new_state;
    };

    Eigen::Vector3d get_state() const {
        return this->state_;
    };


    void set_state_by_value(double x, double y, double z) {
        this->state_(0) = x;
        this->state_(1) = y;
        this->state_(2) = z;
    };

private:
    // state of quadrotor
    Eigen::Vector3d state_;

    // cost-to-goal
    double lmc_;
    double g_cost_;

    // time from start
	double time_{0};
};


}