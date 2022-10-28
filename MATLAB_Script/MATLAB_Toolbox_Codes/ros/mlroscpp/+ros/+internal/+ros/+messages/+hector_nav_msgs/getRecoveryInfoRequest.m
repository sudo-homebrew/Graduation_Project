function [data, info] = getRecoveryInfoRequest
%GetRecoveryInfo gives an empty data for hector_nav_msgs/GetRecoveryInfoRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'hector_nav_msgs/GetRecoveryInfoRequest';
[data.RequestTime, info.RequestTime] = ros.internal.ros.messages.ros.time;
info.RequestTime.MLdataType = 'struct';
[data.RequestRadius, info.RequestRadius] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'hector_nav_msgs/GetRecoveryInfoRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'request_time';
info.MatPath{2} = 'request_time.sec';
info.MatPath{3} = 'request_time.nsec';
info.MatPath{4} = 'request_radius';
