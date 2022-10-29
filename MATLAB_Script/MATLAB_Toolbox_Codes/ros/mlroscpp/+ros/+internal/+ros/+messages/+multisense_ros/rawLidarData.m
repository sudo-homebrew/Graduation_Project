function [data, info] = rawLidarData
%RawLidarData gives an empty data for multisense_ros/RawLidarData

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'multisense_ros/RawLidarData';
[data.ScanCount, info.ScanCount] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.TimeStart, info.TimeStart] = ros.internal.ros.messages.ros.time;
info.TimeStart.MLdataType = 'struct';
[data.TimeEnd, info.TimeEnd] = ros.internal.ros.messages.ros.time;
info.TimeEnd.MLdataType = 'struct';
[data.AngleStart, info.AngleStart] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.AngleEnd, info.AngleEnd] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Distance, info.Distance] = ros.internal.ros.messages.ros.default_type('uint32',NaN);
[data.Intensity, info.Intensity] = ros.internal.ros.messages.ros.default_type('uint32',NaN);
info.MessageType = 'multisense_ros/RawLidarData';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'scan_count';
info.MatPath{2} = 'time_start';
info.MatPath{3} = 'time_start.sec';
info.MatPath{4} = 'time_start.nsec';
info.MatPath{5} = 'time_end';
info.MatPath{6} = 'time_end.sec';
info.MatPath{7} = 'time_end.nsec';
info.MatPath{8} = 'angle_start';
info.MatPath{9} = 'angle_end';
info.MatPath{10} = 'distance';
info.MatPath{11} = 'intensity';
