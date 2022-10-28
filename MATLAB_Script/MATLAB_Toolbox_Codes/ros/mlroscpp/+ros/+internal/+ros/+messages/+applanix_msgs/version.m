function [data, info] = version
%Version gives an empty data for applanix_msgs/Version

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/Version';
[data.Td, info.Td] = ros.internal.ros.messages.applanix_msgs.timeDistance;
info.Td.MLdataType = 'struct';
[data.SystemVersion, info.SystemVersion] = ros.internal.ros.messages.ros.default_type('uint8',120);
[data.PrimaryGnssVersion, info.PrimaryGnssVersion] = ros.internal.ros.messages.ros.default_type('uint8',80);
[data.SecondaryGnssVersion, info.SecondaryGnssVersion] = ros.internal.ros.messages.ros.default_type('uint8',80);
[data.TotalHours, info.TotalHours] = ros.internal.ros.messages.ros.default_type('single',1);
[data.NumRuns, info.NumRuns] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.AvgRunLength, info.AvgRunLength] = ros.internal.ros.messages.ros.default_type('single',1);
[data.LongestRun, info.LongestRun] = ros.internal.ros.messages.ros.default_type('single',1);
[data.CurrentRun, info.CurrentRun] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'applanix_msgs/Version';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'td';
info.MatPath{2} = 'td.time1';
info.MatPath{3} = 'td.time2';
info.MatPath{4} = 'td.distance';
info.MatPath{5} = 'td.time_types';
info.MatPath{6} = 'td.distance_type';
info.MatPath{7} = 'system_version';
info.MatPath{8} = 'primary_gnss_version';
info.MatPath{9} = 'secondary_gnss_version';
info.MatPath{10} = 'total_hours';
info.MatPath{11} = 'num_runs';
info.MatPath{12} = 'avg_run_length';
info.MatPath{13} = 'longest_run';
info.MatPath{14} = 'current_run';
