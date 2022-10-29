function [data, info] = stampedPps
%StampedPps gives an empty data for multisense_ros/StampedPps

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'multisense_ros/StampedPps';
[data.Data, info.Data] = ros.internal.ros.messages.ros.time;
info.Data.MLdataType = 'struct';
[data.HostTime, info.HostTime] = ros.internal.ros.messages.ros.time;
info.HostTime.MLdataType = 'struct';
info.MessageType = 'multisense_ros/StampedPps';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'data';
info.MatPath{2} = 'data.sec';
info.MatPath{3} = 'data.nsec';
info.MatPath{4} = 'host_time';
info.MatPath{5} = 'host_time.sec';
info.MatPath{6} = 'host_time.nsec';
