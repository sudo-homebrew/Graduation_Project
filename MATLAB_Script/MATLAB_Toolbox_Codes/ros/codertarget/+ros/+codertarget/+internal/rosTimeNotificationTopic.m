function topicName = rosTimeNotificationTopic(hObj)
%This function is for internal use only. It may be removed in the future.

%rosTimeNotificationTopic Returns name of the time stepping notification topic

%   Copyright 2018-2019 The MathWorks, Inc.

    cs = hObj.getConfigSet;
    topicName = codertarget.data.getParameterValue(cs, 'ROS.ROSStepNotify');
end
