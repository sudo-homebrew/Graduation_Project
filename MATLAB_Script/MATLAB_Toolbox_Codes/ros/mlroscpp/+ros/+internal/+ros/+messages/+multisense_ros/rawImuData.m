function [data, info] = rawImuData
%RawImuData gives an empty data for multisense_ros/RawImuData

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'multisense_ros/RawImuData';
[data.TimeStamp, info.TimeStamp] = ros.internal.ros.messages.ros.time;
info.TimeStamp.MLdataType = 'struct';
[data.X, info.X] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Y, info.Y] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Z, info.Z] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'multisense_ros/RawImuData';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'time_stamp';
info.MatPath{2} = 'time_stamp.sec';
info.MatPath{3} = 'time_stamp.nsec';
info.MatPath{4} = 'x';
info.MatPath{5} = 'y';
info.MatPath{6} = 'z';
