// Copyright 2021 The MathWorks, Inc.
#ifndef _MLROS2_NODE_H_
#define _MLROS2_NODE_H_

#include "rclcpp/rclcpp.hpp"

// Shared pointer to global node is defined elsewhere
extern rclcpp::Node::SharedPtr gNodePtr;

namespace MATLAB {

inline rclcpp::Node::SharedPtr getGlobalNodeHandle() 
{
    return gNodePtr;
}

}

#endif
