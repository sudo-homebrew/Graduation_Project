classdef CoreInterface < handle
    %CoreInterface Abstract class defining ROS core interface 

    % Copyright 2021 The MathWorks, Inc.
    methods (Access=public, Abstract) 
        isRunning = isCoreRunning(obj);
        stopCore(obj);
        runCore(obj,rosFolder,catkinWorkspace);
    end
end