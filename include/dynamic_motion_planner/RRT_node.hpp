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
        return this->state;
    };
private:
    // data used for KD Tree
    bool kd_in_tree = false;
    bool kd_parent_exist = false;
    bool kd_child_left_exist = false;
    bool kd_child_right_exist = false;
    int kd_split;
    std::shared_ptr<RRTNode> kd_parent;
    std::shared_ptr<RRTNode> kd_child_left;
    std::shared_ptr<RRTNode> kd_child_right;

    // data used for heap in KNN-search
    int heap_index = -1;
    bool in_heap = false;

    //position, the state
    Eigen::Vector3d state;

    // data used for RRTx
    bool in_os_queue = false;
    bool is_move_goal = false;
};


}