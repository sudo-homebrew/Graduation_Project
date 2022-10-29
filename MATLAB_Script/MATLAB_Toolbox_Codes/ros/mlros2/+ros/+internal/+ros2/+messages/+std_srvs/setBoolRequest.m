function [data, info] = setBoolRequest
%SetBool gives an empty data for std_srvs/SetBoolRequest

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'std_srvs/SetBoolRequest';
[data.data, info.data] = ros.internal.ros2.messages.ros2.default_type('logical',1,0);
info.MessageType = 'std_srvs/SetBoolRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'data';
