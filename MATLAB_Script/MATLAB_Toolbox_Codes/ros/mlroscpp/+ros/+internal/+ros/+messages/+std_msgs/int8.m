function [data, info] = int8
%Int8 gives an empty data for std_msgs/Int8

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'std_msgs/Int8';
[data.Data, info.Data] = ros.internal.ros.messages.ros.default_type('int8',1);
info.MessageType = 'std_msgs/Int8';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'data';
