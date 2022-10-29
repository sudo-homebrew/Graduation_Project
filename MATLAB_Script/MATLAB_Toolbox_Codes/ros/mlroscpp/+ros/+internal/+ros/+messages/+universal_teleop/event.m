function [data, info] = event
%Event gives an empty data for universal_teleop/Event

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'universal_teleop/Event';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Event_, info.Event_] = ros.internal.ros.messages.ros.char('string',0);
[data.State, info.State] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'universal_teleop/Event';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'event';
info.MatPath{8} = 'state';
