#pragma once
#include <string>
#include <memory>
#include "jeweler_nav/Variables.hpp"
#include "jeweler_nav/BTLogger.hpp"

namespace jeweler_nav {

class BTNode {
protected:
    std::string name_;
    BlackboardPtr blackboard_;

public:
    BTNode(const std::string& name, BlackboardPtr blackboard) 
    : name_(name), blackboard_(blackboard) {}
    
    virtual ~BTNode() = default;

    // Основной метод обновления логики
    virtual NodeStatus tick() = 0;

    // Метод принудительной остановки/сброса состояния
    virtual void halt() {}

    const std::string& getName() const { return name_; }
};

using NodePtr = std::shared_ptr<BTNode>;

} // namespace jeweler_nav