function [data, info] = tactileArray
%TactileArray gives an empty data for sr_robot_msgs/TactileArray

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/TactileArray';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Data, info.Data] = ros.internal.ros.messages.sr_robot_msgs.tactile;
info.Data.MLdataType = 'struct';
info.Data.MaxLen = NaN;
info.Data.MinLen = 0;
data.Data = data.Data([],1);
info.MessageType = 'sr_robot_msgs/TactileArray';
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
info.MatPath{7} = 'data';
info.MatPath{8} = 'data.data';
info.MatPath{9} = 'data.data.data';
