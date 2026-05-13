#pragma once
#include "jeweler_nav/BTNode.hpp"

namespace jeweler_nav {

class ConditionAlwaysSuccess : public BTNode {
public:
    using BTNode::BTNode;

    NodeStatus tick() override {
        // Узел всегда возвращает успех, позволяя родителю-селектору 
        // считать эту ветку выполненной
        return NodeStatus::SUCCESS;
    }
};

} // namespace jeweler_nav