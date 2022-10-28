function [data, info] = registers
%Registers gives an empty data for robotnik_msgs/Registers

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/Registers';
[data.Registers_, info.Registers_] = ros.internal.ros.messages.robotnik_msgs.register;
info.Registers_.MLdataType = 'struct';
info.Registers_.MaxLen = NaN;
info.Registers_.MinLen = 0;
data.Registers_ = data.Registers_([],1);
info.MessageType = 'robotnik_msgs/Registers';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'registers';
info.MatPath{2} = 'registers.id';
info.MatPath{3} = 'registers.value';
