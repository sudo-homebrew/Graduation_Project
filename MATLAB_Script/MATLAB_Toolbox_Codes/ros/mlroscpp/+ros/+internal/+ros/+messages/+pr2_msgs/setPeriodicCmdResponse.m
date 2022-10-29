function [data, info] = setPeriodicCmdResponse
%SetPeriodicCmd gives an empty data for pr2_msgs/SetPeriodicCmdResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_msgs/SetPeriodicCmdResponse';
[data.StartTime, info.StartTime] = ros.internal.ros.messages.ros.time;
info.StartTime.MLdataType = 'struct';
info.MessageType = 'pr2_msgs/SetPeriodicCmdResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'start_time';
info.MatPath{2} = 'start_time.sec';
info.MatPath{3} = 'start_time.nsec';
