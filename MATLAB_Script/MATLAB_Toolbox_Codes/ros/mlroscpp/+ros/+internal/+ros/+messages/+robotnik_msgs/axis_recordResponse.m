function [data, info] = axis_recordResponse
%axis_record gives an empty data for robotnik_msgs/axis_recordResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/axis_recordResponse';
[data.Ret, info.Ret] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'robotnik_msgs/axis_recordResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'ret';
