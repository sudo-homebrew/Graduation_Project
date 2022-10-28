function [data, info] = ruddersCommand
%RuddersCommand gives an empty data for hector_uav_msgs/RuddersCommand

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'hector_uav_msgs/RuddersCommand';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Aileron, info.Aileron] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Elevator, info.Elevator] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Rudder, info.Rudder] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'hector_uav_msgs/RuddersCommand';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'aileron';
info.MatPath{8} = 'elevator';
info.MatPath{9} = 'rudder';
