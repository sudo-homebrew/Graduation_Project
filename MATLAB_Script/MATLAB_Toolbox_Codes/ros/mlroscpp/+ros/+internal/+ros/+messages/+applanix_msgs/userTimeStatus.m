function [data, info] = userTimeStatus
%UserTimeStatus gives an empty data for applanix_msgs/UserTimeStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/UserTimeStatus';
[data.Td, info.Td] = ros.internal.ros.messages.applanix_msgs.timeDistance;
info.Td.MLdataType = 'struct';
[data.TimeSynchRejections, info.TimeSynchRejections] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.UserTimeResynchs, info.UserTimeResynchs] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.UserTimeValid, info.UserTimeValid] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.TimeSynchMessageReceived, info.TimeSynchMessageReceived] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'applanix_msgs/UserTimeStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'td';
info.MatPath{2} = 'td.time1';
info.MatPath{3} = 'td.time2';
info.MatPath{4} = 'td.distance';
info.MatPath{5} = 'td.time_types';
info.MatPath{6} = 'td.distance_type';
info.MatPath{7} = 'time_synch_rejections';
info.MatPath{8} = 'user_time_resynchs';
info.MatPath{9} = 'user_time_valid';
info.MatPath{10} = 'time_synch_message_received';
