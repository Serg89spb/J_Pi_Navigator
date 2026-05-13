#pragma once
#include "jeweler_nav/BTNode.hpp"

namespace jeweler_nav {

class DecoratorNode : public BTNode {
protected:
    NodePtr child_;

public:
    DecoratorNode(const std::string& name, BlackboardPtr blackboard, NodePtr child)
        : BTNode(name, blackboard), child_(child) {}

    void halt() override {
        if (child_) child_->halt();
    }
};

} // namespace jeweler_nav
