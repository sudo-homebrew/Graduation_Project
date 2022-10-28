function [data, info] = commandLongRequest
%CommandLong gives an empty data for mavros_msgs/CommandLongRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/CommandLongRequest';
[data.Broadcast, info.Broadcast] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Command, info.Command] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.Confirmation, info.Confirmation] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Param1, info.Param1] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Param2, info.Param2] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Param3, info.Param3] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Param4, info.Param4] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Param5, info.Param5] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Param6, info.Param6] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Param7, info.Param7] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'mavros_msgs/CommandLongRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'broadcast';
info.MatPath{2} = 'command';
info.MatPath{3} = 'confirmation';
info.MatPath{4} = 'param1';
info.MatPath{5} = 'param2';
info.MatPath{6} = 'param3';
info.MatPath{7} = 'param4';
info.MatPath{8} = 'param5';
info.MatPath{9} = 'param6';
info.MatPath{10} = 'param7';
