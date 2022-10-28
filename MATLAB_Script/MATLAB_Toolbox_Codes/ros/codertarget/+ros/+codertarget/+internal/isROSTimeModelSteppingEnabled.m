function returnValue = isROSTimeModelSteppingEnabled(hObj)
%This function is for internal use only. It may be removed in the future.

% isROSTimeSteppingEnabled Returns the status of "Enable ROS Time model
% stepping" check box

% Copyright 2019 The MathWorks, Inc.

cs = hObj.getConfigSet;
returnValue = codertarget.data.getParameterValue(cs, 'ROS.ROSTimeStepping');
end