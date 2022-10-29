function [data, info] = status
%Status gives an empty data for bond/Status

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'bond/Status';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Id, info.Id] = ros.internal.ros.messages.ros.char('string',0);
[data.InstanceId, info.InstanceId] = ros.internal.ros.messages.ros.char('string',0);
[data.Active, info.Active] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.HeartbeatTimeout, info.HeartbeatTimeout] = ros.internal.ros.messages.ros.default_type('single',1);
[data.HeartbeatPeriod, info.HeartbeatPeriod] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'bond/Status';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'id';
info.MatPath{8} = 'instance_id';
info.MatPath{9} = 'active';
info.MatPath{10} = 'heartbeat_timeout';
info.MatPath{11} = 'heartbeat_period';
