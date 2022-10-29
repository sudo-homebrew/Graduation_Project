// ROSActionTemplates.hpp
// Copyright 2020-2021 The MathWorks, Inc.

#ifndef ROSACTIONTEMPLATES_H
#define ROSACTIONTEMPLATES_H

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "MATLABROSMsgInterface.hpp"
#include "MATLABActClientInterface.hpp"
#include "MATLABActServerInterface.hpp"

/*
RosActType = actionlib_tutorials::AveragingAction
ActGoalType = actionlib_tutorials::AveragingGoal
RosActFeedbackTypePtr = actionlib_tutorials::AveragingFeedbackConstPtr
RosActResultTypePtr = actionlib_tutorials::AveragingResultConstPtr
GoalCommonType = actionlib_tutorials_msg_AveragingGoal_common
FeedbackCommonType = actionlib_tutorials_msg_AveragingFeedback_common
ResultCommonType = actionlib_tutorials_msg_AveragingResult_common 
*/ 
template<class RosActType, class ActGoalType, class RosActFeedbackTypePtr, class RosActResultTypePtr, class GoalCommonType, class FeedbackCommonType, class ResultCommonType>
class  ROSActClientImpl: public MATLABActClientInterface {
    std::shared_ptr<actionlib::SimpleActionClient<RosActType>> mClient;
    bool isConnectedtoActServer;
    void* mSd;
    SendDataToMATLABFunc_T mOnGoalActiveCBFunc;
    SendDataToMATLABFunc_T mOnFeedbackReceivedCBFunc;
    SendDataToMATLABFunc_T mOnResultReceivedCBFunc;
    intptr_t mHactServer;
    
    GoalCommonType mGoalCommonObj; 
    FeedbackCommonType mFeedbackCommonObj; 
    ResultCommonType mResultCommonObj; 
  public:
    ROSActClientImpl()
        : MATLABActClientInterface(),
          isConnectedtoActServer(false){
    }
    virtual void setCommonObjMap(std::map<std::string,boost::shared_ptr<MATLABROSMsgInterfaceBase>>* commonObjMap){
        mCommonObjMap = commonObjMap;
        mGoalCommonObj.mCommonObjMap = mCommonObjMap;
        mFeedbackCommonObj.mCommonObjMap = mCommonObjMap;
        mResultCommonObj.mCommonObjMap = mCommonObjMap;
    }
    virtual ~ROSActClientImpl() {
    }
    virtual intptr_t createActClient(const std::string& act_name,
                                     std::shared_ptr<ros::NodeHandle> theNode) {
        mClient = std::make_shared<actionlib::SimpleActionClient<RosActType>>(*theNode,act_name,true);
        return reinterpret_cast<intptr_t>(mClient.get());
    }
    virtual bool waitForActServer(intptr_t timeout){
        isConnectedtoActServer = mClient->waitForServer(ros::Duration(timeout,0));
        return isConnectedtoActServer;
    } 
    void onGoalActiveCallback(){
        auto outArray = mFactory.createStructArray({1, 1}, {"goalActive"});
        outArray[0]["goalActive"] = mFactory.createScalar(true);
        if (mSd != NULL) {
            appendAndSendToMATLAB(mSd, mOnGoalActiveCBFunc, outArray, mHactServer);
        }
    }
    void onFeedbackReceivedCallback(const RosActFeedbackTypePtr& feedback){
        auto outArray = mFeedbackCommonObj.get_arr(mFactory, feedback.get(), mMultiLibLoader);
        if (mSd != NULL) {
            appendAndSendToMATLAB(mSd, mOnFeedbackReceivedCBFunc, outArray, mHactServer);
        }
    }
    void onResultReceivedCallback(const actionlib::SimpleClientGoalState& state,
                                         const RosActResultTypePtr& result){
        auto outArray = mResultCommonObj.get_arr(mFactory, result.get(), mMultiLibLoader);
        if (mSd != NULL) {
            appendAndSendToMATLAB(mSd, mOnResultReceivedCBFunc, outArray, mHactServer, static_cast<int32_t>(state.state_));
        }
    }
    virtual bool sendGoaltoActserver( const matlab::data::StructArray arr,
                                      void* sd,
                                      SendDataToMATLABFunc_T onGoalActiveCBFunc,
                                      SendDataToMATLABFunc_T onFeedbackReceivedCBFunc,
                                      SendDataToMATLABFunc_T onResultReceivedCBFunc,
                                      const intptr_t hActClient) {
        //store the function pointers. these will be used in callback;
        mSd = sd;
        mOnGoalActiveCBFunc = onGoalActiveCBFunc;
        mOnFeedbackReceivedCBFunc = onFeedbackReceivedCBFunc;
        mOnResultReceivedCBFunc = onResultReceivedCBFunc;
        mHactServer = hActClient;
        boost::shared_ptr<ActGoalType> goal = boost::make_shared<ActGoalType>();
        mGoalCommonObj.copy_from_struct(goal.get(), arr[0], mMultiLibLoader);
        mClient->sendGoal(*goal,boost::bind(&ROSActClientImpl::onResultReceivedCallback,this,_1,_2),
                               boost::bind(&ROSActClientImpl::onGoalActiveCallback, this),
                               boost::bind(&ROSActClientImpl::onFeedbackReceivedCallback, this,_1));
        return true;
    }
    virtual bool cancelGoal(){
        mClient->cancelGoal();
        return true;
    }
    virtual bool cancelAllGoal(){
        mClient->cancelAllGoals();
        return true;
    }
    virtual bool isServerConnected(){
        isConnectedtoActServer = mClient->isServerConnected();
        return isConnectedtoActServer;
    }
    virtual int32_t getState(){
        auto state = mClient->getState();
        return static_cast<int32_t>(state.state_);
    }
    virtual MDArray_T getResult(){
        auto result = mClient->getResult();
        auto outArray = mResultCommonObj.get_arr(mFactory, result.get(), mMultiLibLoader);
        return outArray;
    }
};

/*
RosActType = actionlib_tutorials::AveragingAction
ActFeedbackType = actionlib_tutorials::AveragingFeedback
ActResultType = actionlib_tutorials::AveragingResult
RosActGoalTypePtr = actionlib_tutorials::AveragingGoalConstPtr
GoalCommonType = actionlib_tutorials_msg_AveragingGoal_common
FeedbackCommonType = actionlib_tutorials_msg_AveragingFeedback_common
ResultCommonType = actionlib_tutorials_msg_AveragingResult_common 
*/ 
template<class RosActType, class ActFeedbackType, class ActResultType, class RosActGoalTypePtr, class GoalCommonType, class FeedbackCommonType, class ResultCommonType>
class ROSActServerImpl : public MATLABActServerInterface {
    
  std::shared_ptr<actionlib::SimpleActionServer<RosActType>> mServer;
  void* mSd;
  SendDataToMATLABFunc_T mNewGoalCBFunc;
  SendDataToMATLABFunc_T mExecuteGoalCBFunc;
  SendDataToMATLABFunc_T mPreemptCBFunc;
  intptr_t mHactServer;
  
  GoalCommonType mGoalCommonObj; 
  FeedbackCommonType mFeedbackCommonObj; 
  ResultCommonType mResultCommonObj; 
  public:
    ROSActServerImpl()
        : MATLABActServerInterface(){
    }
    virtual ~ROSActServerImpl() {
        mServer->shutdown();
    }
    virtual void setCommonObjMap(std::map<std::string,boost::shared_ptr<MATLABROSMsgInterfaceBase>>* commonObjMap){
        mCommonObjMap = commonObjMap;
        mGoalCommonObj.mCommonObjMap = mCommonObjMap;
        mFeedbackCommonObj.mCommonObjMap = mCommonObjMap;
        mResultCommonObj.mCommonObjMap = mCommonObjMap;
    }
    virtual intptr_t addActServer(const std::string& act_name,
                                  bool enableGoalExecCallback,
                                  std::shared_ptr<ros::NodeHandle> theNode,
                                  void* sd,
                                  SendDataToMATLABFunc_T newGoalCBFunc,
                                  SendDataToMATLABFunc_T executeGoalCBFunc,
                                  SendDataToMATLABFunc_T preemptCBFunc,
                                  const intptr_t hSvcServer) {
        mSd = sd;
        mNewGoalCBFunc = newGoalCBFunc;
        mExecuteGoalCBFunc = executeGoalCBFunc;
        mPreemptCBFunc = preemptCBFunc;
        mHactServer = hSvcServer;
        if(enableGoalExecCallback){
            mServer = 
                std::make_shared<actionlib::SimpleActionServer<RosActType>>(
                        *theNode, act_name, boost::bind(&ROSActServerImpl::executeGoalCB, this, _1), false);
        }else{
            mServer = 
                std::make_shared<actionlib::SimpleActionServer<RosActType>>(
                        *theNode, act_name, false);
            mServer->registerGoalCallback(boost::bind(&ROSActServerImpl::newGoalCB,this));
        }
        
        mServer->registerPreemptCallback(boost::bind(&ROSActServerImpl::preemptCB,this));
        mServer->start();
        return reinterpret_cast<intptr_t>(mServer.get());
    }
    void executeGoalCB(const RosActGoalTypePtr &goal){ 
        auto outArray = mGoalCommonObj.get_arr(mFactory, goal.get(), mMultiLibLoader);
        if (mSd != NULL) {
            appendAndSendToMATLAB(mSd, mExecuteGoalCBFunc, outArray, mHactServer);
        }
    }
    void newGoalCB(){
        auto outArray = mFactory.createStructArray({1, 1}, {"newGoal"});
        outArray[0]["newGoal"] = mFactory.createScalar(true);
        if (mSd != NULL) {
            appendAndSendToMATLAB(mSd, mNewGoalCBFunc, outArray, mHactServer);
        }
    }
    void preemptCB(){
        auto outArray = mFactory.createStructArray({1, 1}, {"preemptGoal"});
        outArray[0]["preemptGoal"] = mFactory.createScalar(true);
        if (mSd != NULL) {
            appendAndSendToMATLAB(mSd, mPreemptCBFunc, outArray, mHactServer);
        }
    }

    virtual MDArray_T acceptNewGoal(){
        auto newGoal = mServer->acceptNewGoal();
        auto outArray = mGoalCommonObj.get_arr(mFactory, newGoal.get(), mMultiLibLoader);
        return outArray;
    }
    
    virtual bool isActive(){
        return mServer->isActive();
    }

    virtual bool isNewGoalAvailable(){
        return mServer->isNewGoalAvailable();
    }

    virtual bool isPreemptRequested(){
        return mServer->isPreemptRequested();
    }

    virtual void publishFeedback(const matlab::data::StructArray arr){
        ActFeedbackType feedback;
        mFeedbackCommonObj.copy_from_struct(&feedback, arr[0], mMultiLibLoader);
        mServer->publishFeedback(feedback);
    }

    virtual void setAborted(const matlab::data::StructArray arr, 
                            std::string text){
        ActResultType result;
        mResultCommonObj.copy_from_struct(&result, arr[0], mMultiLibLoader);
        mServer->setAborted(result,text);
    }

    virtual void setPreempted(const matlab::data::StructArray arr, 
                            std::string text){
        ActResultType result;
        mResultCommonObj.copy_from_struct(&result, arr[0], mMultiLibLoader);
        mServer->setPreempted(result,text);
    }

    virtual void setSucceeded(const matlab::data::StructArray arr, 
                            std::string text){
        ActResultType result;
        mResultCommonObj.copy_from_struct(&result, arr[0], mMultiLibLoader);
        mServer->setSucceeded(result,text);
    }
};

#endif // ROSACTIONTEMPLATES_H