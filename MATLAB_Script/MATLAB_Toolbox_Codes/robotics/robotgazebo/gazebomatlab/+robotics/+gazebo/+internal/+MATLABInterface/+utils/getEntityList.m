function modelInfo = getEntityList
%This function is for internal use only. It may be removed in the future.
%
% This function connects with Gazebo server plugin, calls reset Gazebo
% model entity which provides Gazebo model details and close connection.

%   Copyright 2020 The MathWorks, Inc.

    GazeboClient = robotics.internal.GazeboClient;

    % connect to Gazebo
    robotics.gazebo.internal.MATLABInterface.utils.connectClientUsingProfile(GazeboClient);
    
    % get model info
    modelInfo = GazeboClient.getModelEntityList();

    % shutdown client
    GazeboClient.shutdown();

end
