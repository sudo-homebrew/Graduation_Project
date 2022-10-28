function [data, info] = setNamedDigitalOutputResponse
%SetNamedDigitalOutput gives an empty data for robotnik_msgs/SetNamedDigitalOutputResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/SetNamedDigitalOutputResponse';
[data.Ret, info.Ret] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Msg, info.Msg] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'robotnik_msgs/SetNamedDigitalOutputResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'ret';
info.MatPath{2} = 'msg';
