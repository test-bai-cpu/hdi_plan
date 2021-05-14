#include "dynamic_motion_planner/node_list.hpp"

namespace hdi_plan {

Node_list::Node_list() {
    Node end_node;
    end_node.child = &end_node;
    end_node.parent = &end_node;
    this->front = &end_node;
    this->back = &end_node;
    this->bound = &end_node;
    this->length = 0;
}

Node_list::~Node_list() = default;

int Node_list::get_length() {
    return this->length;
}

void Node_list::push_node(RRTNode node, float key) {
    Node new_node;
    new_node.parent = this->front->parent;
    new_node.child = this->front;
    new_node.data = node;
    new_node.key = key;

    if (this->length == 0) {
        this->back = &new_node;
    } else {
        this->front->parent = &new_node;
    }

    this->front = &new_node;
    this->length += 1;
}

}