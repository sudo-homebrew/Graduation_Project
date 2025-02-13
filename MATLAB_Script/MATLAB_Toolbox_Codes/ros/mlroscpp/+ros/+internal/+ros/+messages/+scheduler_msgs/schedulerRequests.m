function [data, info] = schedulerRequests
%SchedulerRequests gives an empty data for scheduler_msgs/SchedulerRequests

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'scheduler_msgs/SchedulerRequests';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Requester, info.Requester] = ros.internal.ros.messages.uuid_msgs.uniqueID;
info.Requester.MLdataType = 'struct';
[data.Requests, info.Requests] = ros.internal.ros.messages.scheduler_msgs.request;
info.Requests.MLdataType = 'struct';
info.Requests.MaxLen = NaN;
info.Requests.MinLen = 0;
data.Requests = data.Requests([],1);
info.MessageType = 'scheduler_msgs/SchedulerRequests';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,50);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'requester';
info.MatPath{8} = 'requester.uuid';
info.MatPath{9} = 'requests';
info.MatPath{10} = 'requests.id';
info.MatPath{11} = 'requests.id.uuid';
info.MatPath{12} = 'requests.resources';
info.MatPath{13} = 'requests.resources.rapp';
info.MatPath{14} = 'requests.resources.id';
info.MatPath{15} = 'requests.resources.id.uuid';
info.MatPath{16} = 'requests.resources.uri';
info.MatPath{17} = 'requests.resources.remappings';
info.MatPath{18} = 'requests.resources.remappings.remap_from';
info.MatPath{19} = 'requests.resources.remappings.remap_to';
info.MatPath{20} = 'requests.resources.parameters';
info.MatPath{21} = 'requests.resources.parameters.key';
info.MatPath{22} = 'requests.resources.parameters.value';
info.MatPath{23} = 'requests.status';
info.MatPath{24} = 'requests.reason';
info.MatPath{25} = 'requests.problem';
info.MatPath{26} = 'requests.NEW';
info.MatPath{27} = 'requests.RESERVED';
info.MatPath{28} = 'requests.WAITING';
info.MatPath{29} = 'requests.GRANTED';
info.MatPath{30} = 'requests.PREEMPTING';
info.MatPath{31} = 'requests.CANCELING';
info.MatPath{32} = 'requests.CLOSED';
info.MatPath{33} = 'requests.NONE';
info.MatPath{34} = 'requests.PREEMPTED';
info.MatPath{35} = 'requests.BUSY';
info.MatPath{36} = 'requests.UNAVAILABLE';
info.MatPath{37} = 'requests.TIMEOUT';
info.MatPath{38} = 'requests.INVALID';
info.MatPath{39} = 'requests.availability';
info.MatPath{40} = 'requests.availability.sec';
info.MatPath{41} = 'requests.availability.nsec';
info.MatPath{42} = 'requests.hold_time';
info.MatPath{43} = 'requests.hold_time.sec';
info.MatPath{44} = 'requests.hold_time.nsec';
info.MatPath{45} = 'requests.priority';
info.MatPath{46} = 'requests.BACKGROUND_PRIORITY';
info.MatPath{47} = 'requests.LOW_PRIORITY';
info.MatPath{48} = 'requests.DEFAULT_PRIORITY';
info.MatPath{49} = 'requests.HIGH_PRIORITY';
info.MatPath{50} = 'requests.CRITICAL_PRIORITY';
