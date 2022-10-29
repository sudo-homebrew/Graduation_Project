// Copyright 2020-2021 The MathWorks, Inc.
#ifndef RATE_HELPER_H
#define RATE_HELPER_H

#include <ros/ros.h>

#define MATLABRate_create(desiredRate) std::unique_ptr<MATLABRate>(new MATLABRate(desiredRate))
#define MATLABRate_isSimTime(obj) obj->isSimTime()
#define MATLABRate_reset(obj) obj->reset()
#define MATLABRate_sleep(obj) obj->sleep()
#define MATLABRate_unused(obj)


class MATLABRate {
  public:
    MATLABRate(double r) : rate(r) {}
    bool isSimTime() {return ::ros::Time::isSimTime();}
    void sleep() {rate.sleep();}
    void reset() {rate.reset();}

  private:
    ::ros::Rate rate;
};

#endif
