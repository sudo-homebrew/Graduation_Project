function isEnabled = isROSTimeNotificationEnabled(hObj)
%This function is for internal use only. It may be removed in the future.

%isROSTimeNotificationEnabled Returns true if the time stepping notification is enabled

%   Copyright 2018-2019 The MathWorks, Inc.

    cs = hObj.getConfigSet;
    isEnabled = codertarget.data.getParameterValue(cs, 'ROS.ROSTimeNotification');
end
