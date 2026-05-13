#include "jeweler_nav/ControlNode.hpp"

namespace jeweler_nav {

class Selector : public ControlNode {
public:
    using ControlNode::ControlNode;

    NodeStatus tick() override {
        for (auto& child : children_) {
            NodeStatus status = child->tick();
            // Если кто-то успешен или в процессе — отдаем наверх
            if (status != NodeStatus::FAILURE) {
                return status;
            }
        }
        return NodeStatus::FAILURE;
    }

    void halt() override {
        for (auto& child : children_) {
            child->halt();
        }
    }
};

} // namespace jeweler_nav
