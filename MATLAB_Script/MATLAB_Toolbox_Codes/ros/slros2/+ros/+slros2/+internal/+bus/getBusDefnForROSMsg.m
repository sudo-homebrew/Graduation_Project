function requiredBuses = getBusDefnForROSMsg(emptyRosMsg, model)
%This function is for internal use only. It may be removed in the future.

%getBusDefnForROSMsg - Create Simulink bus object corresponding to a ROS2
%message
%
%    getBusDefnForROSMsg(MSG) returns an array of Simulink.Bus objects
%    corresponding to ROS message MSG and any nested messages inside it.
%    MSG should be an empty message, since the only way to determine if a
%    property is a variable-length array is to check if its value is [].
%
%    Note:
%    * This function does not create the bus objects in the global scope.
%      Consequently, this function does not define the "name" of the bus
%      (i.e., the name of the bus in the global scope).
%
%    * If the ROS message has variable-size array properties, these are
%      converted to fixed-length arrays (with length of 1), and the
%      associated metadata element is added to the bus object.
%
%    * This function returns a bus object (which is a bus definition), &
%      not an instance of a bus (a bus signal).

%   Copyright 2019-2020 The MathWorks, Inc.

% Good test for this function:
%  msg = ros.slros2.internal.bus.Util.newMsgFromSimulinkMsgType('sensor_msgs/JointState');
%  getBusDefnForROSMsg(msg, 'modelName')

requiredBuses = ros.slros.internal.bus.getBusDefnForROSMsg(emptyRosMsg, model, @ros.slros.internal.bus.defineSimulinkBus);
end
