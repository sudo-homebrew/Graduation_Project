function returnValue = isRemoteBuild(hObj)
%This function is for internal use only. It may be removed in the future.

% isRemoteBuild Returns the value of 'Deploy To' option saved as a boolean
% flag in the Simulink Configuration set

% Copyright 2020-2021 The MathWorks, Inc.

cs = hObj.getConfigSet;
% default setting for ROS 2 is remoteBuild=false
isROS2 = strcmp(get_param(cs, 'HardwareBoard'),...
    message('ros:slros2:codegen:ui_hwboard').getString);
try
    returnValue = codertarget.data.getParameterValue(cs, 'ROS.RemoteBuild');
catch
    returnValue = ~isROS2;
end
end