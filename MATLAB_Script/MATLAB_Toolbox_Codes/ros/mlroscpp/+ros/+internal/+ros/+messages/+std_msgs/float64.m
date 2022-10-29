function [data, info] = float64
%Float64 gives an empty data for std_msgs/Float64

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'std_msgs/Float64';
[data.Data, info.Data] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'std_msgs/Float64';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'data';
