function [data, info] = octomap
%Octomap gives an empty data for octomap_msgs/Octomap

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'octomap_msgs/Octomap';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Binary, info.Binary] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Id, info.Id] = ros.internal.ros.messages.ros.char('string',0);
[data.Resolution, info.Resolution] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Data, info.Data] = ros.internal.ros.messages.ros.default_type('int8',NaN);
info.MessageType = 'octomap_msgs/Octomap';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'binary';
info.MatPath{8} = 'id';
info.MatPath{9} = 'resolution';
info.MatPath{10} = 'data';
