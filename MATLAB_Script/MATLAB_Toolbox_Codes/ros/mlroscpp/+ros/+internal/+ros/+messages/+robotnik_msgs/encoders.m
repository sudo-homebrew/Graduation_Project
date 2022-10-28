function [data, info] = encoders
%encoders gives an empty data for robotnik_msgs/encoders

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/encoders';
[data.Type, info.Type] = ros.internal.ros.messages.ros.char('string',NaN);
[data.Counts, info.Counts] = ros.internal.ros.messages.ros.default_type('int32',NaN);
[data.Vel, info.Vel] = ros.internal.ros.messages.ros.default_type('int32',NaN);
info.MessageType = 'robotnik_msgs/encoders';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'type';
info.MatPath{2} = 'counts';
info.MatPath{3} = 'vel';
