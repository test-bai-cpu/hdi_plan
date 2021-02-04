#pragma once

#include "dynamic_motion_planner/RRT_node.hpp"

namespace hdi_plan {

struct Node{
    Node* child;
    Node* parent;
    RRTNode data;
    float key;
};

class Node_list{
public:
    Node_list();
    ~Node_list();
    int get_length();
    void push_node(RRTNode node, float key);
private:
    Node* front;
    Node* back;
    Node* bound;
    int length;
};



}