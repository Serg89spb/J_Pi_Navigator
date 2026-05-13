#pragma once
#include <vector>
#include "jeweler_nav/BTNode.hpp"

namespace jeweler_nav {

class ControlNode : public BTNode {
protected:
    std::vector<NodePtr> children_;

public:
    explicit ControlNode(const std::string& name, BlackboardPtr blackboard) 
    : BTNode(name, blackboard) {}

    void addChild(NodePtr child) {
        children_.push_back(child);
    }
};

} // namespace jeweler_nav