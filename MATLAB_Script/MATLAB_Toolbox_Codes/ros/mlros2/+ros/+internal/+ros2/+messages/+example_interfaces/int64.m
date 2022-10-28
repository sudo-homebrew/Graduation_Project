function [data, info] = int64
%Int64 gives an empty data for example_interfaces/Int64

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'example_interfaces/Int64';
[data.data, info.data] = ros.internal.ros2.messages.ros2.default_type('int64',1,0);
info.MessageType = 'example_interfaces/Int64';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'data';
