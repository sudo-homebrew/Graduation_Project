function [data, info] = commandIntRequest
%CommandInt gives an empty data for mavros_msgs/CommandIntRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/CommandIntRequest';
[data.Broadcast, info.Broadcast] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Frame, info.Frame] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Command, info.Command] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.Current, info.Current] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Autocontinue, info.Autocontinue] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Param1, info.Param1] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Param2, info.Param2] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Param3, info.Param3] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Param4, info.Param4] = ros.internal.ros.messages.ros.default_type('single',1);
[data.X, info.X] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Y, info.Y] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Z, info.Z] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'mavros_msgs/CommandIntRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'broadcast';
info.MatPath{2} = 'frame';
info.MatPath{3} = 'command';
info.MatPath{4} = 'current';
info.MatPath{5} = 'autocontinue';
info.MatPath{6} = 'param1';
info.MatPath{7} = 'param2';
info.MatPath{8} = 'param3';
info.MatPath{9} = 'param4';
info.MatPath{10} = 'x';
info.MatPath{11} = 'y';
info.MatPath{12} = 'z';
