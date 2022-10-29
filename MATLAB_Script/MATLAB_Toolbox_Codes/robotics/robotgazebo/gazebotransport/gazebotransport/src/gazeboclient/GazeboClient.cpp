/* Copyright 2019-2021 The MathWorks, Inc. */
#include "gazebotransport/GazeboClient.hpp"
#include "gazebotransport/gazeboclient/GazeboClientImpl.hpp"
#include "gazebotransport/gazeboclient/SubscribeGenericMessageAction.hpp"
#include "gazebotransport/gazeboclient/GetGenericMessageAction.hpp"
#include "gazebotransport/gazeboclient/StepSimulationAction.hpp"
#include "gazebotransport/gazeboclient/ResetSimulationAction.hpp"
#include "gazebotransport/gazeboclient/RequestCoSimAction.hpp"
#include "gazebotransport/gazeboclient/StopCoSimAction.hpp"
#include "gazebotransport/gazeboclient/GetGroundTruthWorldPoseAction.hpp"
#include "gazebotransport/gazeboclient/ApplyJointTorqueAction.hpp"
#include "gazebotransport/gazeboclient/ApplyLinkWrenchAction.hpp"
#include "gazebotransport/gazeboclient/GetTopicListAction.hpp"
#include "gazebotransport/gazeboclient/GetModelInfoAction.hpp"
#include "gazebotransport/gazeboclient/GetMaxStepSizeAction.hpp"
#include "gazebotransport/gazeboclient/SubscribeCustomMessageAction.hpp"
#include "gazebotransport/gazeboclient/PublishCustomMessageAction.hpp"
#include "gazebotransport/gazeboclient/InitPublishCustomMessageAction.hpp"
#include "gazebotransport/gazeboclient/InitSubscribeCustomMessageAction.hpp"
#include "gazebotransport/gazeboclient/SetJointPositionAction.hpp"
#include "gazebotransport/gazeboclient/SetJointVelocityAction.hpp"
#include "gazebotransport/gazeboclient/GetJointStateAction.hpp"
#include "gazebotransport/gazeboclient/SetLinkWorldPoseAction.hpp"
#include "gazebotransport/gazeboclient/SetLinkLinearVelocityAction.hpp"
#include "gazebotransport/gazeboclient/SetLinkAngularVelocityAction.hpp"
#include "gazebotransport/gazeboclient/GetLinkStateAction.hpp"
#include "gazebotransport/gazeboclient/SetGazeboModelParamAction.hpp"
#include "gazebotransport/gazeboclient/GetGazeboModelParamAction.hpp"
#include "gazebotransport/gazeboclient/GetGazeboModelSDFAction.hpp"

#include "boost/asio.hpp"
#include "boost/date_time.hpp"
#include <future>
#include <mutex>
#include <cmath>

namespace robotics {
namespace gazebotransport {

class Heartbeat {
  public:
    Heartbeat(double interval)
        : m_service()
        , m_work(std::make_shared<boost::asio::io_service::work>(m_service))
        , m_timer(m_service)
        , m_result()
        , m_mutex()
        , m_interval(interval) {
        // launch the service queue in a different thread to keep the deadline timer ticking
        m_result = std::async(std::launch::async, [this]() { this->m_service.run(); });
    }

    ~Heartbeat() {
        std::lock_guard<std::mutex> lock(m_mutex);
        // reset timer
        boost::system::error_code ignored;
        m_timer.cancel(ignored);
        // reset work place holder
        m_work.reset();
        // wait the run to finish
        m_result.get();
    }

    bool beat(std::shared_ptr<GazeboClientImpl> client,
              std::shared_ptr<RequestCoSimAction> action) {
        client->act(*action);
        bool ret = action->success();

        if (ret) {
            // continue heartbeat if server accepts last request
            std::lock_guard<std::mutex> lock(m_mutex);
            if (std::isfinite(m_interval)) {
                m_timer.expires_from_now(
                    boost::posix_time::milliseconds(static_cast<int64_t>(m_interval)));
                m_timer.async_wait([this, client, action](const boost::system::error_code& ec) {
                    if (!ec) {
                        beat(client, action);
                    }
                });
            }
        }

        return ret;
    }

    void stop() {
        boost::system::error_code ignored;
        std::lock_guard<std::mutex> lock(m_mutex);
        m_timer.cancel(ignored);
    }

  private:
    /// Boost asio queue
    boost::asio::io_service m_service;

    /// Place holder to keep service running
    std::shared_ptr<boost::asio::io_service::work> m_work;

    /// Boost deadline timer
    boost::asio::deadline_timer m_timer;

    /// Boost asio queue run result
    std::future<void> m_result;

    std::mutex m_mutex;

    double m_interval;
};

GazeboClient::GazeboClient(std::string const& ipaddress,
                           uint16_t port,
                           uint64_t timeout,
                           double pulseTime)
    : m_clientImpl(std::make_shared<GazeboClientImpl>(ipaddress, port, timeout))
    , m_heartbeatInterval(pulseTime)
    , m_heartbeatTimeout(pulseTime * 5)
    , m_heartbeat(std::make_shared<Heartbeat>(m_heartbeatInterval))
    , m_actions()
    , m_simulationTime({0, 0}) {
    // fill in the Gazebo actions
    m_actions[GazeboClient::ActionMapID::SUBSCRIBE_GENERIC_MESSAGE_ACTION] =
        std::make_shared<SubscribeGenericMessageAction>();
    m_actions[GazeboClient::ActionMapID::RESET_SIMULATION_ACTION] =
        std::make_shared<ResetSimulationAction>();
    m_actions[GazeboClient::ActionMapID::STEP_SIMULATION_ACTION] =
        std::make_shared<StepSimulationAction>();
    m_actions[GazeboClient::ActionMapID::REQUEST_COSIM_ACTION] =
        std::make_shared<RequestCoSimAction>();
    m_actions[GazeboClient::ActionMapID::STOP_COSIM_ACTION] = std::make_shared<StopCoSimAction>();
    m_actions[GazeboClient::ActionMapID::APPLY_JOINT_TORQUE] =
        std::make_shared<ApplyJointTorqueAction>();
    m_actions[GazeboClient::ActionMapID::APPLY_LINK_WRENCH] =
        std::make_shared<ApplyLinkWrenchAction>();
    m_actions[GazeboClient::ActionMapID::GET_TOPIC_LIST] = std::make_shared<GetTopicListAction>();
    m_actions[GazeboClient::ActionMapID::GET_MODEL_INFO] = std::make_shared<GetModelInfoAction>();
    m_actions[GazeboClient::ActionMapID::GET_MAX_STEP_SIZE] =
        std::make_shared<GetMaxStepSizeAction>();
    m_actions[GazeboClient::ActionMapID::PUBLISH_CUSTOM_MESSAGE] =
        std::make_shared<PublishCustomMessageAction>();
    m_actions[GazeboClient::ActionMapID::SUBSCRIBE_CUSTOM_MESSAGE] =
        std::make_shared<SubscribeCustomMessageAction>();
    m_actions[GazeboClient::ActionMapID::INIT_CUSTOM_MESSAGE_PUBLISH] =
        std::make_shared<InitPublishCustomMessageAction>();
    m_actions[GazeboClient::ActionMapID::INIT_CUSTOM_MESSAGE_SUBSCRIBE] =
        std::make_shared<InitSubscribeCustomMessageAction>();
    m_actions[GazeboClient::ActionMapID::SET_JOINT_POSITION] =
        std::make_shared<SetJointPositionAction>();
    m_actions[GazeboClient::ActionMapID::SET_JOINT_VELOCITY] =
        std::make_shared<SetJointVelocityAction>();
    m_actions[GazeboClient::ActionMapID::GET_JOINT_STATE] = std::make_shared<GetJointStateAction>();
    m_actions[GazeboClient::ActionMapID::SET_LINK_WORLD_POSE] =
        std::make_shared<SetLinkWorldPoseAction>();
    m_actions[GazeboClient::ActionMapID::SET_LINK_LINEAR_VELOCITY] =
        std::make_shared<SetLinkLinearVelocityAction>();
    m_actions[GazeboClient::ActionMapID::SET_LINK_ANGULAR_VELOCITY] =
        std::make_shared<SetLinkAngularVelocityAction>();
    m_actions[GazeboClient::ActionMapID::GET_LINK_STATE] = std::make_shared<GetLinkStateAction>();
    m_actions[GazeboClient::ActionMapID::SET_GAZEBO_MODEL_PARAM] =
        std::make_shared<SetGazeboModelParamAction>();
    m_actions[GazeboClient::ActionMapID::GET_GAZEBO_MODEL_PARAM] =
        std::make_shared<GetGazeboModelParamAction>();
    m_actions[GazeboClient::ActionMapID::GET_GAZEBO_MODEL_SDF] =
        std::make_shared<GetGazeboModelSDFAction>();
}

bool GazeboClient::requestCoSim(std::string const& clientID) {
    // perform request co-simulation action
    auto action = std::dynamic_pointer_cast<RequestCoSimAction>(
        m_actions[GazeboClient::ActionMapID::REQUEST_COSIM_ACTION]);
    action->setRequest(clientID, m_heartbeatTimeout);
    return m_heartbeat->beat(m_clientImpl, action);
}

bool GazeboClient::stopCoSim(std::string const& clientID) {

    // stop heartbeat
    m_heartbeat->stop();

    // perform stop co-simulation action
    auto action = std::dynamic_pointer_cast<StopCoSimAction>(
        m_actions[GazeboClient::ActionMapID::STOP_COSIM_ACTION]);
    action->setClientID(clientID);
    m_clientImpl->act(*action);

    return action->success();
}

bool GazeboClient::stepSimulation(uint32_t numSteps) {
    // perform step simulation action
    auto action = std::dynamic_pointer_cast<StepSimulationAction>(
        m_actions[GazeboClient::ActionMapID::STEP_SIMULATION_ACTION]);
    action->setStepSize(numSteps);
    m_clientImpl->act(*action);
    return action->success();
}

bool GazeboClient::subscribeGenericMessage(std::string const& topicType,
                                           std::string const& topicName) {
    // perform subscribe to message action
    auto action = std::dynamic_pointer_cast<SubscribeGenericMessageAction>(
        m_actions[GazeboClient::ActionMapID::SUBSCRIBE_GENERIC_MESSAGE_ACTION]);
    if (action->setTopic(topicType, topicName)) {
        m_clientImpl->act(*action);
        bool ret = action->success();

        // if subscribe is successful, create a new subscriber
        if (ret) {
            m_subscribers[topicName] = std::make_shared<GetGenericMessageAction>();
        }

        return ret;
    } else {
        return false;
    }
}

std::pair<bool, std::shared_ptr<mw::internal::robotics::gazebotransport::Packet>>
GazeboClient::getGenericMessage(std::string const& topicType, std::string const& topicName) {
    // try get the corresponding subscriber
    auto subscriberIter = m_subscribers.find(topicName);
    if (subscriberIter == m_subscribers.end()) {
        return std::make_pair(false, nullptr);
    }

    // perform get message action
    auto action = std::dynamic_pointer_cast<GetGenericMessageAction>(subscriberIter->second);
    action->setTopic(topicType, topicName);
    action->setSimulationTime(m_simulationTime.seconds, m_simulationTime.nanoSeconds);
    m_clientImpl->act(*action);
    return action->getMessage();
}

bool GazeboClient::resetTime() {
    auto action = std::dynamic_pointer_cast<ResetSimulationAction>(
        m_actions[GazeboClient::ActionMapID::RESET_SIMULATION_ACTION]);
    action->setResetMode(
        mw::internal::robotics::gazebotransport::ResetSimulation_ResetBehavior_RESET_TIME);
    m_clientImpl->act(*action);
    return action->success();
}

bool GazeboClient::resetAll() {
    auto action = std::dynamic_pointer_cast<ResetSimulationAction>(
        m_actions[GazeboClient::ActionMapID::RESET_SIMULATION_ACTION]);
    action->setResetMode(mw::internal::robotics::gazebotransport::
                             ResetSimulation_ResetBehavior_RESET_TIME_AND_SCENE);
    m_clientImpl->act(*action);
    return action->success();
}

void GazeboClient::shutdown() {
    m_heartbeat->stop();
    m_clientImpl->shutdown();
}

void GazeboClient::setSimulationTime(uint64_t seconds, uint64_t nanoSeconds) {
    m_simulationTime.seconds = seconds;
    m_simulationTime.nanoSeconds = nanoSeconds;
}



std::pair<bool, std::shared_ptr<mw::internal::robotics::gazebotransport::Packet>>
GazeboClient::getGroundTruthWorldPose(std::string const& modelName, std::string const& linkName) {
    std::shared_ptr<GetGroundTruthWorldPoseAction> subscriberPtr = nullptr;
    auto topicName = modelName + linkName;

    auto subscriberIter = m_subscribers.find(topicName);
    if (subscriberIter == m_subscribers.end()) {
        // create new subscriber if it doesn't already exist
        subscriberPtr = std::make_shared<GetGroundTruthWorldPoseAction>();
        m_subscribers.emplace(std::make_pair(topicName, subscriberPtr));
    } else {
        subscriberPtr =
            std::dynamic_pointer_cast<GetGroundTruthWorldPoseAction>(subscriberIter->second);
    }

    subscriberPtr->setLinkName(modelName, linkName);
    subscriberPtr->setSimulationTime(m_simulationTime.seconds, m_simulationTime.nanoSeconds);
    m_clientImpl->act(*subscriberPtr);
    return subscriberPtr->getLastPose();
}

bool GazeboClient::applyJointTorque(std::string const& modelName,
                                    std::string const& jointName,
                                    uint32_t indexValue,
                                    double effortValue,
                                    uint64_t durationSeconds,
                                    uint64_t durationNanoSeconds) {
    auto action = std::dynamic_pointer_cast<ApplyJointTorqueAction>(
        m_actions[GazeboClient::ActionMapID::APPLY_JOINT_TORQUE]);
    action->setJointTorque(modelName, jointName, indexValue, effortValue, durationSeconds,
                           durationNanoSeconds);
    m_clientImpl->act(*action);
    return action->success();
}

bool GazeboClient::applyLinkWrench(std::string const& modelName,
                                   std::string const& linkName,
                                   std::string const& forceType,
                                   std::vector<double> const& force,
                                   std::string const& torqueType,
                                   std::vector<double> const& torque,
                                   uint64_t durationSeconds,
                                   uint64_t durationNanoSeconds) {
    auto action = std::dynamic_pointer_cast<ApplyLinkWrenchAction>(
        m_actions[GazeboClient::ActionMapID::APPLY_LINK_WRENCH]);
    action->setLinkWrench(modelName, linkName, forceType, force, torqueType, torque,
                          durationSeconds, durationNanoSeconds);
    m_clientImpl->act(*action);
    return action->success();
}


std::shared_ptr<mw::internal::robotics::gazebotransport::Packet> GazeboClient::getTopicList() {
    auto action = std::dynamic_pointer_cast<GetTopicListAction>(
        m_actions[GazeboClient::ActionMapID::GET_TOPIC_LIST]);
    m_clientImpl->act(*action);
    return action->getTopicList();
}

std::shared_ptr<mw::internal::robotics::gazebotransport::Packet> GazeboClient::getModelInfo() {
    auto action = std::dynamic_pointer_cast<GetModelInfoAction>(
        m_actions[GazeboClient::ActionMapID::GET_MODEL_INFO]);
    m_clientImpl->act(*action);
    return action->getModelInfo();
}

double GazeboClient::getMaxStepSize() {
    auto action = std::dynamic_pointer_cast<GetMaxStepSizeAction>(
        m_actions[GazeboClient::ActionMapID::GET_MAX_STEP_SIZE]);
    m_clientImpl->act(*action);
    return action->getMaxStepSize();
}


bool GazeboClient::publishCustomMessage(std::string const& topicName,
                                        std::string const& messageType,
                                        std::string const& customMsgString) {
    // publish custom message action
    auto action = std::dynamic_pointer_cast<PublishCustomMessageAction>(
        m_actions[GazeboClient::ActionMapID::PUBLISH_CUSTOM_MESSAGE]);

    action->setCustomMsg(topicName, messageType, customMsgString);

    m_clientImpl->act(*action);
    return action->success();
}


bool GazeboClient::initPublishCustomMessage(std::string const& topicName,
                                            std::string const& messageType) {
    // initialize custom message publisher action
    auto action = std::dynamic_pointer_cast<InitPublishCustomMessageAction>(
        m_actions[GazeboClient::ActionMapID::INIT_CUSTOM_MESSAGE_PUBLISH]);

    action->setCustomMsg(topicName, messageType);

    m_clientImpl->act(*action);
    return action->success();
}

bool GazeboClient::initSubscribeCustomMessage(std::string const& topicName,
                                              std::string const& messageType) {
    // initialize custom message subscriber action
    auto action = std::dynamic_pointer_cast<InitSubscribeCustomMessageAction>(
        m_actions[GazeboClient::ActionMapID::INIT_CUSTOM_MESSAGE_SUBSCRIBE]);

    action->setCustomMsg(topicName, messageType);

    m_clientImpl->act(*action);
    return action->success();
}

std::pair<bool, std::shared_ptr<mw::internal::robotics::gazebotransport::Packet>>
GazeboClient::subscribeCustomMessage(std::string const& topicName, std::string const& messageType) {
    // subscribe custom message action
    auto action = std::dynamic_pointer_cast<SubscribeCustomMessageAction>(
        m_actions[GazeboClient::ActionMapID::SUBSCRIBE_CUSTOM_MESSAGE]);

    action->setCustomMsg(topicName, messageType);

    m_clientImpl->act(*action);
    return action->getMessage();
}

bool GazeboClient::setJointPosition(std::string const& modelName,
                                    std::string const& jointName,
                                    uint32_t indexValue,
                                    double position,
                                    uint64_t durationSeconds,
                                    uint64_t durationNanoSeconds) {
    auto action = std::dynamic_pointer_cast<SetJointPositionAction>(
        m_actions[GazeboClient::ActionMapID::SET_JOINT_POSITION]);
    action->setJointPosition(modelName, jointName, indexValue, position, durationSeconds,
                             durationNanoSeconds);
    m_clientImpl->act(*action);
    return action->success();
}

bool GazeboClient::setJointVelocity(std::string const& modelName,
                                    std::string const& jointName,
                                    uint32_t indexValue,
                                    double velocity,
                                    uint64_t durationSeconds,
                                    uint64_t durationNanoSeconds) {
    auto action = std::dynamic_pointer_cast<SetJointVelocityAction>(
        m_actions[GazeboClient::ActionMapID::SET_JOINT_VELOCITY]);
    action->setJointVelocity(modelName, jointName, indexValue, velocity, durationSeconds,
                             durationNanoSeconds);
    m_clientImpl->act(*action);
    return action->success();
}

std::pair<bool, std::shared_ptr<mw::internal::robotics::gazebotransport::Packet>>
GazeboClient::getJointState(std::string const& modelName, std::string const& jointName) {
    std::shared_ptr<GetJointStateAction> subscriberPtr = nullptr;
    auto topicName = modelName + jointName;

    auto subscriberIter = m_subscribers.find(topicName);
    if (subscriberIter == m_subscribers.end()) {
        // create new subscriber if it doesn't already exist
        subscriberPtr = std::make_shared<GetJointStateAction>();
        m_subscribers.emplace(std::make_pair(topicName, subscriberPtr));
    } else {
        subscriberPtr = std::dynamic_pointer_cast<GetJointStateAction>(subscriberIter->second);
    }

    subscriberPtr->setJointName(modelName, jointName);
    subscriberPtr->setSimulationTime(m_simulationTime.seconds, m_simulationTime.nanoSeconds);
    m_clientImpl->act(*subscriberPtr);
    return subscriberPtr->getLastJointState();
}

std::pair<bool, std::shared_ptr<mw::internal::robotics::gazebotransport::Packet>>
GazeboClient::getLinkState(std::string const& modelName, std::string const& linkName) {
    std::shared_ptr<GetLinkStateAction> subscriberPtr = nullptr;
    auto topicName = modelName + linkName;

    auto subscriberIter = m_subscribers.find(topicName);
    if (subscriberIter == m_subscribers.end()) {
        // create new subscriber if it doesn't already exist
        subscriberPtr = std::make_shared<GetLinkStateAction>();
        m_subscribers.emplace(std::make_pair(topicName, subscriberPtr));
    } else {
        subscriberPtr = std::dynamic_pointer_cast<GetLinkStateAction>(subscriberIter->second);
    }

    subscriberPtr->setLinkName(modelName, linkName);
    subscriberPtr->setSimulationTime(m_simulationTime.seconds, m_simulationTime.nanoSeconds);
    m_clientImpl->act(*subscriberPtr);
    return subscriberPtr->getLastLinkState();
}

bool GazeboClient::setLinkWorldPose(std::string const& modelName,
                                    std::string const& linkName,
                                    std::vector<double> const& pose,
                                    uint64_t durationSeconds,
                                    uint64_t durationNanoSeconds) {
    auto action = std::dynamic_pointer_cast<SetLinkWorldPoseAction>(
        m_actions[GazeboClient::ActionMapID::SET_LINK_WORLD_POSE]);
    action->setLinkWorldPose(modelName, linkName, pose, durationSeconds, durationNanoSeconds);
    m_clientImpl->act(*action);
    return action->success();
}

bool GazeboClient::setLinkLinearVelocity(std::string const& modelName,
                                         std::string const& linkName,
                                         std::vector<double> const& velocity,
                                         uint64_t durationSeconds,
                                         uint64_t durationNanoSeconds) {
    auto action = std::dynamic_pointer_cast<SetLinkLinearVelocityAction>(
        m_actions[GazeboClient::ActionMapID::SET_LINK_LINEAR_VELOCITY]);
    action->setLinkLinearVelocity(modelName, linkName, velocity, durationSeconds,
                                  durationNanoSeconds);
    m_clientImpl->act(*action);
    return action->success();
}

bool GazeboClient::setLinkAngularVelocity(std::string const& modelName,
                                          std::string const& linkName,
                                          std::vector<double> const& velocity,
                                          uint64_t durationSeconds,
                                          uint64_t durationNanoSeconds) {
    auto action = std::dynamic_pointer_cast<SetLinkAngularVelocityAction>(
        m_actions[GazeboClient::ActionMapID::SET_LINK_ANGULAR_VELOCITY]);
    action->setLinkAngularVelocity(modelName, linkName, velocity, durationSeconds,
                                   durationNanoSeconds);
    m_clientImpl->act(*action);
    return action->success();
}

std::pair<bool, std::shared_ptr<mw::internal::robotics::gazebotransport::Packet>>
GazeboClient::getGazeboModelParam(std::string const& modelName,
                                  bool isLink,
                                  std::string const& linkJointName) {

    auto action = std::dynamic_pointer_cast<GetGazeboModelParamAction>(
        m_actions[GazeboClient::ActionMapID::GET_GAZEBO_MODEL_PARAM]);

    action->setModelName(modelName, isLink, linkJointName);
    action->setSimulationTime(m_simulationTime.seconds, m_simulationTime.nanoSeconds);
    m_clientImpl->act(*action);

    return action->getGazeboModelParam();
}

std::pair<uint8_t, std::string> GazeboClient::setGazeboModelParam(
    mw::internal::robotics::gazebotransport::Gazebomodel& message) {
    auto action = std::dynamic_pointer_cast<SetGazeboModelParamAction>(
        m_actions[GazeboClient::ActionMapID::SET_GAZEBO_MODEL_PARAM]);

    action->setGazeboModelParam(message);
    m_clientImpl->act(*action);

    return action->success();
}

std::pair<bool, std::shared_ptr<mw::internal::robotics::gazebotransport::Packet>>
GazeboClient::getGazeboModelSDF(std::string const& modelName) {

    auto action = std::dynamic_pointer_cast<GetGazeboModelSDFAction>(
        m_actions[GazeboClient::ActionMapID::GET_GAZEBO_MODEL_SDF]);

    action->setModelName(modelName);
    action->setSimulationTime(m_simulationTime.seconds, m_simulationTime.nanoSeconds);
    m_clientImpl->act(*action);

    return action->getGazeboModelSDF();
}

} // namespace gazebotransport
} // namespace robotics
