#include "jeweler_nav/ControlNode.hpp"

namespace jeweler_nav {

class Sequence : public ControlNode {
public:
    using ControlNode::ControlNode;

    NodeStatus tick() override {
        for (auto& child : children_) {
            NodeStatus status = child->tick();
            // Если ребенок еще работает или упал — прерываемся
            if (status != NodeStatus::SUCCESS) {
                return status;
            }
        }
        return NodeStatus::SUCCESS;
    }

    void halt() override {
        for (auto& child : children_) {
            child->halt();
        }
    }
};

} // namespace jeweler_nav