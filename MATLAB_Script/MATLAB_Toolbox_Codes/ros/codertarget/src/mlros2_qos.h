// Copyright 2021 The MathWorks, Inc.
#ifndef _MLROS2_QOS_H
#define _MLROS2_QOS_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

/**
* Macro to set QOS Values
* @param qosStruct QOS profile structure
* @param hist history setting.
* @param dep depth which contains queue size.
* @param dur durability setting.
* @param rel reliablity setting
*/
#ifndef SET_QOS_VALUES
#define SET_QOS_VALUES(qosStruct, hist, dep, dur, rel)  \
    {                                                   \
        qosStruct.history = hist;                       \
        qosStruct.depth = dep;                          \
        qosStruct.durability = dur;                     \
        qosStruct.reliability = rel;                    \
    }
#endif

// Get QOS Settings from RMW
inline rclcpp::QoS getQOSSettingsFromRMW(const rmw_qos_profile_t& qosProfile) {
    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(qosProfile));
    if (RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL == qosProfile.durability) {
        qos.transient_local();
    } else {
        qos.durability_volatile();
    }
    if (RMW_QOS_POLICY_RELIABILITY_RELIABLE == qosProfile.reliability) {
        qos.reliable();
    } else {
        qos.best_effort();
    }
    return qos;
}

#endif
