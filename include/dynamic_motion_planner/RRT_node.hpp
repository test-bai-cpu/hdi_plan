#pragma once

#include <Eigen/Dense>
#include <memory>
namespace hdi_plan {

class RRTNode {
public:
    RRTNode() = default;
    RRTNode(Eigen::Vector3d state);
    ~RRTNode();

    Eigen::Vector3d get_state() const {
        return this->state_;
    };

    double get_cost() const {
        return this->cost_;
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
    bool in_os_queue = false;
    bool is_move_goal = false;

    // cost related
    double cost_;


};


}