function [data, info] = request
%Request gives an empty data for scheduler_msgs/Request

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'scheduler_msgs/Request';
[data.Id, info.Id] = ros.internal.ros.messages.uuid_msgs.uniqueID;
info.Id.MLdataType = 'struct';
[data.Resources, info.Resources] = ros.internal.ros.messages.scheduler_msgs.resource;
info.Resources.MLdataType = 'struct';
info.Resources.MaxLen = NaN;
info.Resources.MinLen = 0;
data.Resources = data.Resources([],1);
[data.Status, info.Status] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Reason, info.Reason] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Problem, info.Problem] = ros.internal.ros.messages.ros.char('string',0);
[data.NEW, info.NEW] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.RESERVED, info.RESERVED] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.WAITING, info.WAITING] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.GRANTED, info.GRANTED] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.PREEMPTING, info.PREEMPTING] = ros.internal.ros.messages.ros.default_type('uint8',1, 4);
[data.CANCELING, info.CANCELING] = ros.internal.ros.messages.ros.default_type('uint8',1, 5);
[data.CLOSED, info.CLOSED] = ros.internal.ros.messages.ros.default_type('uint8',1, 6);
[data.NONE, info.NONE] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.PREEMPTED, info.PREEMPTED] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.BUSY, info.BUSY] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.UNAVAILABLE, info.UNAVAILABLE] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.TIMEOUT, info.TIMEOUT] = ros.internal.ros.messages.ros.default_type('uint8',1, 4);
[data.INVALID, info.INVALID] = ros.internal.ros.messages.ros.default_type('uint8',1, 5);
[data.Availability, info.Availability] = ros.internal.ros.messages.ros.time;
info.Availability.MLdataType = 'struct';
[data.HoldTime, info.HoldTime] = ros.internal.ros.messages.ros.duration;
info.HoldTime.MLdataType = 'struct';
[data.Priority, info.Priority] = ros.internal.ros.messages.ros.default_type('int16',1);
[data.BACKGROUNDPRIORITY, info.BACKGROUNDPRIORITY] = ros.internal.ros.messages.ros.default_type('int16',1, -20000);
[data.LOWPRIORITY, info.LOWPRIORITY] = ros.internal.ros.messages.ros.default_type('int16',1, -10000);
[data.DEFAULTPRIORITY, info.DEFAULTPRIORITY] = ros.internal.ros.messages.ros.default_type('int16',1, 0);
[data.HIGHPRIORITY, info.HIGHPRIORITY] = ros.internal.ros.messages.ros.default_type('int16',1, 10000);
[data.CRITICALPRIORITY, info.CRITICALPRIORITY] = ros.internal.ros.messages.ros.default_type('int16',1, 20000);
info.MessageType = 'scheduler_msgs/Request';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,41);
info.MatPath{1} = 'id';
info.MatPath{2} = 'id.uuid';
info.MatPath{3} = 'resources';
info.MatPath{4} = 'resources.rapp';
info.MatPath{5} = 'resources.id';
info.MatPath{6} = 'resources.id.uuid';
info.MatPath{7} = 'resources.uri';
info.MatPath{8} = 'resources.remappings';
info.MatPath{9} = 'resources.remappings.remap_from';
info.MatPath{10} = 'resources.remappings.remap_to';
info.MatPath{11} = 'resources.parameters';
info.MatPath{12} = 'resources.parameters.key';
info.MatPath{13} = 'resources.parameters.value';
info.MatPath{14} = 'status';
info.MatPath{15} = 'reason';
info.MatPath{16} = 'problem';
info.MatPath{17} = 'NEW';
info.MatPath{18} = 'RESERVED';
info.MatPath{19} = 'WAITING';
info.MatPath{20} = 'GRANTED';
info.MatPath{21} = 'PREEMPTING';
info.MatPath{22} = 'CANCELING';
info.MatPath{23} = 'CLOSED';
info.MatPath{24} = 'NONE';
info.MatPath{25} = 'PREEMPTED';
info.MatPath{26} = 'BUSY';
info.MatPath{27} = 'UNAVAILABLE';
info.MatPath{28} = 'TIMEOUT';
info.MatPath{29} = 'INVALID';
info.MatPath{30} = 'availability';
info.MatPath{31} = 'availability.sec';
info.MatPath{32} = 'availability.nsec';
info.MatPath{33} = 'hold_time';
info.MatPath{34} = 'hold_time.sec';
info.MatPath{35} = 'hold_time.nsec';
info.MatPath{36} = 'priority';
info.MatPath{37} = 'BACKGROUND_PRIORITY';
info.MatPath{38} = 'LOW_PRIORITY';
info.MatPath{39} = 'DEFAULT_PRIORITY';
info.MatPath{40} = 'HIGH_PRIORITY';
info.MatPath{41} = 'CRITICAL_PRIORITY';