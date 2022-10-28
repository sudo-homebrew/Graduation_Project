/* Copyright 2018 The MathWorks, Inc. */

#ifndef _SLROS_TIME_H_
#define _SLROS_TIME_H_

#include <iostream>
#include <ros/ros.h>

/**
 * Retrieve the current ROS time and return the information in a message bus.
 *
 * @param busPtr[out] Simulink bus structure that should be populated with time
 */
template <class BusType>
inline void currentROSTimeBus(BusType* busPtr) {
    ros::Time currentTime = ros::Time::now();
    convertToBus(busPtr, &currentTime);
}

/**
 * Retrieve the current ROS time and return as a double.
 * This method is separate from @c currentROSTimeBus, since it has a completely
 * different signature.
 *
 * @return Current time as double-precision value (in seconds)
 */
inline void currentROSTimeDouble(double* timePtr) {
    ros::Time currentTime = ros::Time::now();
    *timePtr = currentTime.toSec();
}

#endif
