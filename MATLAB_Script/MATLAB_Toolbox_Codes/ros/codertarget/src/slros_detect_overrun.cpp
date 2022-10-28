/* Copyright 2019 The MathWorks, Inc. */
#include "slros_initialize.h"

void reportOverrun (int rateID) {
    static uint32_t overrunCnt = 0;
    overrunCnt++;
    ROS_ERROR("Overrun %d at rate %d", overrunCnt, rateID);
}
