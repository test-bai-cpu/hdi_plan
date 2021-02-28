#pragma once

#include <Eigen/Dense>
#include <memory>
#include <ompl/datastructures/BinaryHeap.h>

#include "dynamic_motion_planner/motion_planner.hpp"

namespace hdi_plan {

class RRTNode {
public:
    RRTNode() = default;
    RRTNode(Eigen::Vector3d state);
    ~RRTNode();

    Eigen::Vector3d get_state() const {
        return this->state_;
    };

    double get_lmc() const {
        return this->lmc_;
    };

    double set_lmc(double lmc) {
        this->lmc_ = lmc;
    };

    double get_g_cost() const {
        return this->g_cost_;
    };

    double set_g_cost(double g_cost) {
        this->g_cost_ = g_cost;
    };

    Eigen::Vector3d set_state_by_vector(Eigen::Vector3d new_state) {
        this->state_ = new_state;
    };
    Eigen::Vector3d set_state_by_value(double x, double y, double z) {
        this->state_(0) = x;
        this->state_(1) = y;
        this->state_(2) = z;
    };

    std::shared_ptr<RRTNode> parent;
    std::vector<std::shared_ptr<RRTNode>> children;

    // for extend part
    std::vector<std::pair<std::shared_ptr<RRTNode>, bool>> nbh;

    // for neighbors, original in is N0-(v), out is N0+(v), running in is Nr-(v), out is Nr+(v)
    std::vector<std::shared_ptr<RRTNode>> n0_in; // -
    std::vector<std::shared_ptr<RRTNode>> n0_out; // +
    std::vector<std::shared_ptr<RRTNode>> nr_in; // -
    std::vector<std::shared_ptr<RRTNode>> nr_out; // +

    bool in_queue = false;

    ompl::BinaryHeap<std::shared_ptr<RRTNode>, MotionPlanner::node_compare>::Element *handle;


private:
    // data used for KD Tree
    bool kd_in_tree = false;
    bool kd_parent_exist = false;
    bool kd_child_left_exist = false;
    bool kd_child_right_exist = false;
    int kd_split;


    // data used for heap in KNN-search
    int heap_index = -1;
    bool in_heap = false;

    //position, the state
    Eigen::Vector3d state_;

    // data used for RRTx
    bool in_os_queue_ = false;
    bool is_move_goal_ = false;

    // cost related
    double cost_ = 0;
    double inc_cost_ = 0;
    double lmc_ = 0;
    double g_cost_ = 0;



};


}