// Copyright 2021 The MathWorks, Inc.
#ifndef _MLROSCPP_TRANSFORM_H
#define _MLROSCPP_TRANSFORM_H

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "ros_structmsg_conversion.h" // For struct2msg()

#include <string>
#include <algorithm> // For std::sort

#define UNUSED_PARAM(x)

class MATLABROSTransform {
  public:
    MATLABROSTransform() {
        mNodeHandle = std::make_shared<ros::NodeHandle>();
        // Set the default buffer time to 10s
        setCacheTime(10, 0);
    };

    /**
     * Get the transform between two frames by frame ID.
     * @param transformStampedStruct message struct to be passed to MATLAB class
     * @param mlTargetFrame target frame name
     * @param mlTargetFrameSize size of target frame name
     * @param mlSourceFrame source frame name
     * @param mlSourceFrameSize size of source frame name
     * @param targetTimeSec sec field of target time
     * @param targetTimeNsec nSec field of target time
     * @param timeoutSec sec field of timeout
     * @param timeoutNsec nSec field of timeout
     */
    void lookupTransform(geometry_msgs_TransformStampedStruct_T* transformStampedStruct,
                         const char* mlTargetFrame,
                         size_t mlTargetFrameSize,
                         const char* mlSourceFrame,
                         size_t mlSourceFrameSize,
                         uint32_t targetTimeSec,
                         uint32_t targetTimeNsec,
                         uint32_t timeoutSec,
                         uint32_t timeoutNsec) {
        // Check if listener and buffer are initialized. Else throw ROS_ERROR
        if (!mTfBuffer.get() || !mTfListener.get()) {
            ROS_ERROR("Transformation-Listener is not initialized properly.");
        }

        std::string targetFrame(mlTargetFrame, mlTargetFrameSize);
        std::string sourceFrame(mlSourceFrame, mlSourceFrameSize);

        // Create an empty tf message
        mTfStructPtr = std::make_shared<geometry_msgs_TransformStampedStruct_T>();

        // Return empty transformStamped message if there is no valid transformation in Network
        try {
            geometry_msgs::TransformStamped tfStampedMsg = mTfBuffer->lookupTransform(
                targetFrame, sourceFrame, ros::Time(targetTimeSec, targetTimeNsec),
                ros::Duration(timeoutSec, timeoutNsec));
            msg2struct(mTfStructPtr.get(), &tfStampedMsg);
        } catch (...) {
        }

        *transformStampedStruct = *mTfStructPtr;
    }

    /**
     * Send a StampedTransform including frame_id, parent_id, and time.
     * @param transformStampedStruct message struct from MATLAB class
     */
    void sendTransform(const geometry_msgs_TransformStampedStruct_T transformStampedStruct) {
        const geometry_msgs_TransformStampedStruct_T* structPtr = &transformStampedStruct;
        mTfMsgPtr = std::make_shared<geometry_msgs::TransformStamped>();
        struct2msg(mTfMsgPtr.get(), structPtr);
        mTfBroadcaster.sendTransform(*mTfMsgPtr);
    }

    /**
     * Test if a transform is possible.
     * @param mlTargetFrame target frame name
     * @param mlTargetFrameSize size of target frame name
     * @param mlSourceFrame source frame name
     * @param mlSourceFrameSize size of source frame name
     * @param targetTimeSec sec field of target time
     * @param targetTimeNsec nSec field of target time
     */
    bool canTransform(const char* mlTargetFrame,
                      size_t mlTargetFrameSize,
                      const char* mlSourceFrame,
                      size_t mlSourceFrameSize,
                      uint32_t targetTimeSec,
                      uint32_t targetTimeNsec) {

        std::string targetFrame(mlTargetFrame, mlTargetFrameSize);
        std::string sourceFrame(mlSourceFrame, mlSourceFrameSize);

        bool isAva =
            mTfBuffer->canTransform(targetFrame, sourceFrame,
                                    ros::Time(targetTimeSec, targetTimeNsec), ros::Duration(0, 0));

        return isAva;
    }

    /**
     * Get the cache time of the transformation buffer
     * @param sec sec field of cache time
     * @param nSec nSec field of cache time
     */
    bool getCacheTime(uint32_t* sec, uint32_t* nSec) {
        if (mTfBuffer.get() && mTfListener.get()) {
            ros::Duration d = mTfBuffer->getCacheLength();
            *sec = d.sec;
            *nSec = d.nsec;
            return true;
        }
        return false;
    }

    /**
     * Set the cache time of the transformation buffer
     * @param sec sec field of cache time
     * @param nSec nSec field of cache time
     */
    void setCacheTime(uint32_t sec, uint32_t nSec) {
        uint32_t currentSec, currentNsec;
        if (getCacheTime(&currentSec, &currentNsec) && (currentSec == sec) &&
            (currentNsec == nSec)) {
            // if everything is equal, return;
            return;
        }

        // Otherwise, create new buffer and new listener with the new cache time
        ros::Duration d(sec, nSec);
        mTfBuffer = std::make_shared<tf2_ros::Buffer>(d);
        mTfListener = std::make_shared<tf2_ros::TransformListener>(*mTfBuffer, *mNodeHandle);
    }

    /**
     * Update and get number of available frames from ROS network
     */
    uint32_t updateAndGetNumOfFrames() {
        // Get name of available frames from network
        if (mTfBuffer.get() && mTfListener.get()) {
            mTfBuffer->_getFrameStrings(mframeNames);
            std::sort(mframeNames.begin(), mframeNames.end());
            return mframeNames.size();
        } else {
            return 0;
        }
    }

    /**
     * Get the length of specific frame
     * @param key index of specific frame
     */
    int getFrameNameLength(int key) {
        return mframeNames[key].size();
    }

    /**
     * Get the specific frame name given frame index
     * @param key index of specific frame
     * @param frameEntry name of the frame
     */
    void getAvailableFrame(int key, char* frameEntry) {
        if (mframeNames.size() > 0) {
            const char* frameStr = mframeNames[key].c_str();
            size_t frameNameLen = getFrameNameLength(key);
            strncpy(frameEntry, frameStr, frameNameLen);
            frameEntry[frameNameLen] = 0;
        } else {
            frameEntry[0] = 0;
        }
    }

  private:
    std::shared_ptr<ros::NodeHandle> mNodeHandle;
    std::shared_ptr<tf2_ros::Buffer> mTfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> mTfListener;
    tf2_ros::TransformBroadcaster mTfBroadcaster;
    std::shared_ptr<geometry_msgs::TransformStamped> mTfMsgPtr;
    std::shared_ptr<geometry_msgs_TransformStampedStruct_T> mTfStructPtr;
    std::vector<std::string> mframeNames{};
};



#endif
