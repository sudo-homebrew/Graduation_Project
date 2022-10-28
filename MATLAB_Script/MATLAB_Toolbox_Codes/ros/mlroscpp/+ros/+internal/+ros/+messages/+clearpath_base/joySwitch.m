function [data, info] = joySwitch
%JoySwitch gives an empty data for clearpath_base/JoySwitch

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'clearpath_base/JoySwitch';
[data.RobotId, info.RobotId] = ros.internal.ros.messages.ros.char('string',0);
[data.Attach, info.Attach] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Joystick, info.Joystick] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'clearpath_base/JoySwitch';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'robot_id';
info.MatPath{2} = 'attach';
info.MatPath{3} = 'joystick';
