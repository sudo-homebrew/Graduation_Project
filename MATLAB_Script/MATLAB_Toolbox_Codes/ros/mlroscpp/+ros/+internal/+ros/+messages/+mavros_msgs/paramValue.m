function [data, info] = paramValue
%ParamValue gives an empty data for mavros_msgs/ParamValue

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/ParamValue';
[data.Integer, info.Integer] = ros.internal.ros.messages.ros.default_type('int64',1);
[data.Real, info.Real] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'mavros_msgs/ParamValue';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'integer';
info.MatPath{2} = 'real';
