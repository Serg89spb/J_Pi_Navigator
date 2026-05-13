#pragma once
#include "jeweler_nav/BTNode.hpp"

namespace jeweler_nav {

// Листовой узел (Action или Condition)
class LeafNode : public BTNode {
public:
    using BTNode::BTNode;
    // Здесь позже добавим методы доступа к данным сенсоров
};

} // namespace jeweler_nav