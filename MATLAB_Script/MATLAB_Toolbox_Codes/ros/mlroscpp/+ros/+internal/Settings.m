classdef Settings
    %This class is for internal use only. It may be removed in the future.
    
    %SETTINGS Gathering various settings in one class
    
    %   Copyright 2020 The MathWorks, Inc.
    
    properties (Constant)
        %SimulationTimeParameter - ROS parameter name that indicates simulation time
        SimulationTimeParameter = '/use_sim_time'
        
        %ClockTopic - The topic that is used to publish time messages
        ClockTopic = '/clock'
        
        %ClockMessageType - The message type for time messages
        ClockMessageType = 'rosgraph_msgs/Clock'
        
        %TfTopic - ROS topic name that is used for TF messages
        TfTopic = '/tf'
        
        %TfMessageType - Message type for /tf topic in TF1
        TfMessageType = 'tf/tfMessage'
        
        %Tf2MessageType - Message type for /tf topic in TF2
        Tf2MessageType = 'tf2_msgs/TFMessage'
        
        %DefaultMasterConnectionTimeout - Timeout (in secs) for master connection
        %   This is a default value that should be used for any connections
        %   to the ROS Master.
        DefaultMasterConnectionTimeout = 5.0
        
        %DefaultMasterRegistrationTimeout - Timeout (in secs) for registering ROS entities with the Master
        %   This is a default value that should be used for any ROS Master
        %   registrations.
        DefaultMasterRegistrationTimeout = 5.0
    end
    
end

