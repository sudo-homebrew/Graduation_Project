function [data, info] = controllerInfo
%ControllerInfo gives an empty data for kobuki_msgs/ControllerInfo

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'kobuki_msgs/ControllerInfo';
[data.DEFAULT, info.DEFAULT] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.USERCONFIGURED, info.USERCONFIGURED] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.Type, info.Type] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.PGain, info.PGain] = ros.internal.ros.messages.ros.default_type('double',1);
[data.IGain, info.IGain] = ros.internal.ros.messages.ros.default_type('double',1);
[data.DGain, info.DGain] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'kobuki_msgs/ControllerInfo';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'DEFAULT';
info.MatPath{2} = 'USER_CONFIGURED';
info.MatPath{3} = 'type';
info.MatPath{4} = 'p_gain';
info.MatPath{5} = 'i_gain';
info.MatPath{6} = 'd_gain';
