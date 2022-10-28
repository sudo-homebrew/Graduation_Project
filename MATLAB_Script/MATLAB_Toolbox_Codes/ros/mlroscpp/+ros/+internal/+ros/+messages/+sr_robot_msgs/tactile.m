function [data, info] = tactile
%Tactile gives an empty data for sr_robot_msgs/Tactile

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/Tactile';
[data.Data, info.Data] = ros.internal.ros.messages.std_msgs.int16;
info.Data.MLdataType = 'struct';
info.Data.MaxLen = NaN;
info.Data.MinLen = 0;
data.Data = data.Data([],1);
info.MessageType = 'sr_robot_msgs/Tactile';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'data';
info.MatPath{2} = 'data.data';
