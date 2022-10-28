function connectClientUsingProfile(GazeboClient)
%This function is for internal use only. It may be removed in the future.
%
% This function connects to Gazebo server with stored connection settings

%   Copyright 2020 The MathWorks, Inc.

    % connect to Gazebo
    profileStore = robotics.gazebo.internal.MATLABInterface.sim.GazeboPreferenceStore;
    profile = profileStore.getProfile();
    % profile.SimulationTimeout is in 'second' but backend code needs
    % 'msec'
    GazeboClient.connect(profile.MasterHost, profile.MasterPort, profile.SimulationTimeout*1000);
    
end

