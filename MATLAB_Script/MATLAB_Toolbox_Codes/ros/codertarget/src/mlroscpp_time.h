// Copyright 2020 The MathWorks, Inc.
#ifndef _MLROSCPP_TIME_H_
#define _MLROSCPP_TIME_H_

#include "ros/time.h"

template <class MsgStructType>
bool time2struct(MsgStructType *msgStruct, bool isSystemTime)
{
    bool isSimTime;
    if (isSystemTime) {
        // Use wall clock
        ros::WallTime currentTime = ros::WallTime::now();
        msgStruct->Sec = currentTime.sec;
        msgStruct->Nsec = currentTime.nsec;
        isSimTime = false;
    }
    else {
        // Use ROS clock
        ros::Time currentTime = ros::Time::now();
        msgStruct->Sec = currentTime.sec;
        msgStruct->Nsec = currentTime.nsec;
        isSimTime = ros::Time::isSimTime();
    }
    
    return isSimTime;
}

#endif
