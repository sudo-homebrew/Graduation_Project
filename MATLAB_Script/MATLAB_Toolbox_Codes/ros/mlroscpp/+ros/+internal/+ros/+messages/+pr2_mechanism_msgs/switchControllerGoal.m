function [data, info] = switchControllerGoal
%SwitchControllerGoal gives an empty data for pr2_mechanism_msgs/SwitchControllerGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_mechanism_msgs/SwitchControllerGoal';
[data.StartControllers, info.StartControllers] = ros.internal.ros.messages.ros.char('string',NaN);
[data.StopControllers, info.StopControllers] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'pr2_mechanism_msgs/SwitchControllerGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'start_controllers';
info.MatPath{2} = 'stop_controllers';
