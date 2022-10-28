function [data, info] = wFOVTrigger
%WFOVTrigger gives an empty data for wfov_camera_msgs/WFOVTrigger

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'wfov_camera_msgs/WFOVTrigger';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.TimeReference, info.TimeReference] = ros.internal.ros.messages.ros.char('string',0);
[data.TriggerTime, info.TriggerTime] = ros.internal.ros.messages.ros.time;
info.TriggerTime.MLdataType = 'struct';
[data.TriggerTimeReference, info.TriggerTimeReference] = ros.internal.ros.messages.ros.char('string',0);
[data.Shutter, info.Shutter] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Id, info.Id] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.TriggerSeq, info.TriggerSeq] = ros.internal.ros.messages.ros.default_type('uint32',1);
info.MessageType = 'wfov_camera_msgs/WFOVTrigger';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'time_reference';
info.MatPath{8} = 'trigger_time';
info.MatPath{9} = 'trigger_time.sec';
info.MatPath{10} = 'trigger_time.nsec';
info.MatPath{11} = 'trigger_time_reference';
info.MatPath{12} = 'shutter';
info.MatPath{13} = 'id';
info.MatPath{14} = 'trigger_seq';
