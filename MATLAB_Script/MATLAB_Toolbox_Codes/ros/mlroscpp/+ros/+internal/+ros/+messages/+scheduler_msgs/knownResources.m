function [data, info] = knownResources
%KnownResources gives an empty data for scheduler_msgs/KnownResources

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'scheduler_msgs/KnownResources';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Resources, info.Resources] = ros.internal.ros.messages.scheduler_msgs.currentStatus;
info.Resources.MLdataType = 'struct';
info.Resources.MaxLen = NaN;
info.Resources.MinLen = 0;
data.Resources = data.Resources([],1);
info.MessageType = 'scheduler_msgs/KnownResources';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,17);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'resources';
info.MatPath{8} = 'resources.uri';
info.MatPath{9} = 'resources.status';
info.MatPath{10} = 'resources.AVAILABLE';
info.MatPath{11} = 'resources.ALLOCATED';
info.MatPath{12} = 'resources.MISSING';
info.MatPath{13} = 'resources.GONE';
info.MatPath{14} = 'resources.owner';
info.MatPath{15} = 'resources.owner.uuid';
info.MatPath{16} = 'resources.priority';
info.MatPath{17} = 'resources.rapps';
