function [data, info] = assembleScansResponse
%AssembleScans gives an empty data for laser_assembler/AssembleScansResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'laser_assembler/AssembleScansResponse';
[data.Cloud, info.Cloud] = ros.internal.ros.messages.sensor_msgs.pointCloud;
info.Cloud.MLdataType = 'struct';
info.MessageType = 'laser_assembler/AssembleScansResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'cloud';
info.MatPath{2} = 'cloud.header';
info.MatPath{3} = 'cloud.header.seq';
info.MatPath{4} = 'cloud.header.stamp';
info.MatPath{5} = 'cloud.header.stamp.sec';
info.MatPath{6} = 'cloud.header.stamp.nsec';
info.MatPath{7} = 'cloud.header.frame_id';
info.MatPath{8} = 'cloud.points';
info.MatPath{9} = 'cloud.points.x';
info.MatPath{10} = 'cloud.points.y';
info.MatPath{11} = 'cloud.points.z';
info.MatPath{12} = 'cloud.channels';
info.MatPath{13} = 'cloud.channels.name';
info.MatPath{14} = 'cloud.channels.values';
