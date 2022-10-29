function [data, info] = setPeriodicCmdRequest
%SetPeriodicCmd gives an empty data for pr2_msgs/SetPeriodicCmdRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_msgs/SetPeriodicCmdRequest';
[data.Command, info.Command] = ros.internal.ros.messages.pr2_msgs.periodicCmd;
info.Command.MLdataType = 'struct';
info.MessageType = 'pr2_msgs/SetPeriodicCmdRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'command';
info.MatPath{2} = 'command.header';
info.MatPath{3} = 'command.header.seq';
info.MatPath{4} = 'command.header.stamp';
info.MatPath{5} = 'command.header.stamp.sec';
info.MatPath{6} = 'command.header.stamp.nsec';
info.MatPath{7} = 'command.header.frame_id';
info.MatPath{8} = 'command.profile';
info.MatPath{9} = 'command.period';
info.MatPath{10} = 'command.amplitude';
info.MatPath{11} = 'command.offset';
