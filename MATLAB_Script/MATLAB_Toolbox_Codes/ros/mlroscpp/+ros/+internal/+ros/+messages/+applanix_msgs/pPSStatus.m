function [data, info] = pPSStatus
%PPSStatus gives an empty data for applanix_msgs/PPSStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/PPSStatus';
[data.Td, info.Td] = ros.internal.ros.messages.applanix_msgs.timeDistance;
info.Td.MLdataType = 'struct';
[data.PpsCount, info.PpsCount] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.STATUSNONE, info.STATUSNONE] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.STATUSSYNCING, info.STATUSSYNCING] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.STATUSSYNCED, info.STATUSSYNCED] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.STATUSOLDOFFSET, info.STATUSOLDOFFSET] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.Status, info.Status] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'applanix_msgs/PPSStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'td';
info.MatPath{2} = 'td.time1';
info.MatPath{3} = 'td.time2';
info.MatPath{4} = 'td.distance';
info.MatPath{5} = 'td.time_types';
info.MatPath{6} = 'td.distance_type';
info.MatPath{7} = 'pps_count';
info.MatPath{8} = 'STATUS_NONE';
info.MatPath{9} = 'STATUS_SYNCING';
info.MatPath{10} = 'STATUS_SYNCED';
info.MatPath{11} = 'STATUS_OLD_OFFSET';
info.MatPath{12} = 'status';
