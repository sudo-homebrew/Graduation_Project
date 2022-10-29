classdef Constants 
%This function is for internal use only. It may be removed in the future.

%   Copyright 2019 The MathWorks, Inc.

% Store product-wide constants and preferences in this class

    % This class should only contain the common constants, preference names
    % and values that are used product wide. Constant values related to a
    % particular task (for example, supported cmake_version for
    % ColconBuilder) should be stored in tool-specific classes.

    properties (Constant)
        %PreferenceGroup - The group name for MATLAB preference storage
        PreferenceGroup = 'ROS_Toolbox'
    end
end
