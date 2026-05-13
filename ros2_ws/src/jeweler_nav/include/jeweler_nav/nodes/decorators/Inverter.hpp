#pragma once
#include "jeweler_nav/nodes/DecoratorNode.hpp"

namespace jeweler_nav {

class Inverter : public DecoratorNode {
public:
    using DecoratorNode::DecoratorNode;

    NodeStatus tick() override {
        if (!child_) return NodeStatus::FAILURE;

        NodeStatus status = child_->tick();

        if (status == NodeStatus::SUCCESS) return NodeStatus::FAILURE;
        if (status == NodeStatus::FAILURE) return NodeStatus::SUCCESS;
        
        return status; // RUNNING остается RUNNING
    }
};

} // namespace jeweler_nav