// Copyright 2020-2021 The MathWorks, Inc.
#ifndef _MLROSCPP_ACTCLIENT_H_
#define _MLROSCPP_ACTCLIENT_H_

#include <iostream>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h> // For actionlib
#include <actionlib/client/terminal_state.h>       // For actionlib
#include "ros_structmsg_conversion.h"              // For msg2struct()
#include <string.h>                                // For std::string
#include <algorithm>                               // For std::tolower
#include <functional>                              // For std::function
#include <mutex>

#define MATLABActClient_lock(obj) obj->lock()
#define MATLABActClient_unlock(obj) obj->unlock()
#define MATLABActClient_updateGoalState(obj) obj->updateGoalState()
#define MATLABActClient_createActClient(obj, mlActName, mlActSize) \
    obj->createActClient(mlActName, mlActSize)
#define MATLABActClient_getStateLength(obj) obj->getStateLength()
#define MATLABActClient_getStatusTextLength(obj) obj->getStatusTextLength()
#define MATLABActClient_getStatusInfo(obj, mlState, mlStatusText) \
    obj->getStatusInfo(mlState, mlStatusText)
#define MATLABActClient_waitForServer(obj, timeout, status) obj->waitForServer(timeout, status)
#define MATLABActClient_sendGoal(obj, goalMsgStruct) obj->sendGoal(goalMsgStruct)
#define MATLABActClient_sendGoalAndWait(obj, goalMsgStruct, timeout, isDone) \
    obj->sendGoalAndWait(goalMsgStruct, timeout, isDone)
#define MATLABActClient_cancelAllGoals(obj) obj->cancelAllGoals()
#define MATLABActClient_cancelGoal(obj) obj->cancelGoal()

template <class ActionType,
          class GoalMsgType,
          class FbMsgType,
          class ResultMsgType,
          class GoalStructType,
          class FbStructType,
          class ResultStructType>
class MATLABActClient {
  private:
    std::function<void(void)> activationCallback_;
    std::function<void(void)> feedbackCallback_;
    std::function<void(void)> resultCallback_;
    std::mutex mutex_;

    boost::shared_ptr<actionlib::SimpleActionClient<ActionType>> client_;
    GoalStructType* goalStructPtr_;
    FbStructType* feedbackStructPtr_;
    ResultStructType* resultStructPtr_;
    GoalMsgType goalMsg_;
    boost::shared_ptr<const ResultMsgType> resultMsg_;
    std::string mlState_;
    std::string mlStatusText_;

  public:
    MATLABActClient(std::function<void(void)> activationCallback,
                    std::function<void(void)> feedbackCallback,
                    std::function<void(void)> resultCallback,
                    GoalStructType* goalStructPtr,
                    FbStructType* feedbackStructPtr,
                    ResultStructType* resultStructPtr)
        : activationCallback_{activationCallback}
        , feedbackCallback_{feedbackCallback}
        , resultCallback_{resultCallback}
        , goalStructPtr_{goalStructPtr}
        , feedbackStructPtr_{feedbackStructPtr}
        , resultStructPtr_{resultStructPtr} {}

    /**
     * Creates an action client and register it on the ROS network.
     * @param mlActName - action client name retrieved from MATLAB
     * @param mlActSize - the length of the action client name
     */
    void createActClient(const char* mlActName, size_t mlActSize) {
        std::string actname(mlActName, mlActSize);
        client_.reset(new actionlib::SimpleActionClient<ActionType>(actname, true));
    }

    /**
     * Waits for the action server to connect to this client.
     * @param timeout - user specified timeout for waiting.
     * @param status - bool pointer to indicate existence.
     */
    void waitForServer(double timeout, bool* status) {
        if (timeout > 0) {
            *status = client_->waitForServer(ros::Duration(timeout));
        } else {
            *status = client_->waitForServer();
        }
    }

    /**
     * Sends a goal to the action server, and also registers callbacks.
     * @param goalMsgStruct - goal message struct
     */
    void sendGoal(GoalStructType goalMsgStruct) {
        const GoalStructType* goalStructPtr = &goalMsgStruct;
        mutex_.lock();
        struct2msg(&goalMsg_, goalStructPtr);
        mutex_.unlock();
        client_->sendGoal(
            goalMsg_,
            boost::bind(&MATLABActClient<ActionType, GoalMsgType, FbMsgType, ResultMsgType,
                                         GoalStructType, FbStructType, ResultStructType>::resultCb,
                        this, _1, _2),
            boost::bind(
                &MATLABActClient<ActionType, GoalMsgType, FbMsgType, ResultMsgType, GoalStructType,
                                 FbStructType, ResultStructType>::activationCb,
                this),
            boost::bind(
                &MATLABActClient<ActionType, GoalMsgType, FbMsgType, ResultMsgType, GoalStructType,
                                 FbStructType, ResultStructType>::feedbackCb,
                this, _1));
    }

    /**
     * Get the result message of the sent goal from action server
     */
    void getResult() {
        client_->waitForResult();
        resultMsg_ = client_->getResult();
        mutex_.lock();
        msg2struct(resultStructPtr_, resultMsg_.get());
        mutex_.unlock();
    }

    /**
     * Send a goal to the action server, and waits until the goal completes or a timeout is
     * exceeded.
     * @param goalMsgStruct - goal message struct
     * @param timeout - timeout for waiting goal to be executed
     */
    void sendGoalAndWait(GoalStructType goalMsgStruct, double timeout, bool* isDone) {
        const GoalStructType* goalStructPtr = &goalMsgStruct;
        mutex_.lock();
        struct2msg(&goalMsg_, goalStructPtr);
        mutex_.unlock();
        client_->sendGoal(
            goalMsg_,
            boost::bind(&MATLABActClient<ActionType, GoalMsgType, FbMsgType, ResultMsgType,
                                         GoalStructType, FbStructType, ResultStructType>::resultCb,
                        this, _1, _2),
            boost::bind(
                &MATLABActClient<ActionType, GoalMsgType, FbMsgType, ResultMsgType, GoalStructType,
                                 FbStructType, ResultStructType>::activationCb,
                this),
            boost::bind(
                &MATLABActClient<ActionType, GoalMsgType, FbMsgType, ResultMsgType, GoalStructType,
                                 FbStructType, ResultStructType>::feedbackCb,
                this, _1));
        *isDone = client_->waitForResult(ros::Duration(timeout));
        updateGoalState();
        resultMsg_ = client_->getResult();
        mutex_.lock();
        msg2struct(resultStructPtr_, resultMsg_.get());
        mutex_.unlock();
    }

    /**
     * update the goal status
     */
    void updateGoalState() {
        actionlib::SimpleClientGoalState state(actionlib::SimpleClientGoalState::LOST);
        state = client_->getState();
        mlState_ = state.toString();
        std::transform(mlState_.begin(), mlState_.end(), mlState_.begin(),
                       [](unsigned char c) { return ::tolower(c); });
        mlStatusText_ = state.getText();
    }

    /**
     * retrieve the length of the status
     */
    size_t getStateLength() {
        return mlState_.length();
    }

    /**
     * retrieve the length of state text
     */
    size_t getStatusTextLength() {
        return mlStatusText_.length();
    }

    /**
     * retrieve the status and state text of current goal
     * @param mlState - status pointer to save the status information
     * @param mlStatusText - state text pointer to save the state text
     */
    void getStatusInfo(char* mlState, char* mlStatusText) {
        const char* cState_ = mlState_.c_str();
        size_t mlStateLen = getStateLength();
        std::strncpy(mlState, cState_, mlStateLen);
        mlState[mlStateLen] = 0;

        const char* cStatusText_ = mlStatusText_.c_str();
        size_t mlStatusTextLen = getStatusTextLength();
        std::strncpy(mlStatusText, cStatusText_, mlStatusTextLen);
        mlStatusText[mlStatusTextLen] = 0;
    }

    /**
     * Cancel the goal that we are currently pursuing.
     */
    void cancelGoal() {
        client_->cancelGoal();
    }

    /**
     * Cancel all goals currently running on the action server.
     */
    void cancelAllGoals() {
        client_->cancelAllGoals();
    }

    /**
     * Mutex lock to avoid read and write conflict on messages
     */
    void lock() {
        mutex_.lock();
    }

    /**
     * Mutex unlock to release mutex lock
     */
    void unlock() {
        mutex_.unlock();
    }

  private:
    /**
     * Result callback function for sendGoal
     * @param state - the state of current goal
     * @param result - pointer to result message
     */
    void resultCb(const actionlib::SimpleClientGoalState& state,
                  boost::shared_ptr<const ResultMsgType> result) {
        mutex_.lock();
        msg2struct(resultStructPtr_, result.get());
        mutex_.unlock();
        resultCallback_();
    }

    /**
     * Activation callback function for sendGoal
     */
    void activationCb() {
        activationCallback_();
    }

    /**
     * Feedback callback function for sendGoal
     * @param feedback - pointer to feedback message
     */
    void feedbackCb(boost::shared_ptr<const FbMsgType> feedback) {
        mutex_.lock();
        msg2struct(feedbackStructPtr_, feedback.get());
        mutex_.unlock();
        feedbackCallback_();
    }
};

#endif
