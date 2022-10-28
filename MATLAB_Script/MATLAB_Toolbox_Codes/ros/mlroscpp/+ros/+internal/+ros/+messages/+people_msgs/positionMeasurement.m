function [data, info] = positionMeasurement
%PositionMeasurement gives an empty data for people_msgs/PositionMeasurement

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'people_msgs/PositionMeasurement';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.ObjectId, info.ObjectId] = ros.internal.ros.messages.ros.char('string',0);
[data.Pos, info.Pos] = ros.internal.ros.messages.geometry_msgs.point;
info.Pos.MLdataType = 'struct';
[data.Reliability, info.Reliability] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Covariance, info.Covariance] = ros.internal.ros.messages.ros.default_type('double',9);
[data.Initialization, info.Initialization] = ros.internal.ros.messages.ros.default_type('int8',1);
info.MessageType = 'people_msgs/PositionMeasurement';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,15);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'name';
info.MatPath{8} = 'object_id';
info.MatPath{9} = 'pos';
info.MatPath{10} = 'pos.x';
info.MatPath{11} = 'pos.y';
info.MatPath{12} = 'pos.z';
info.MatPath{13} = 'reliability';
info.MatPath{14} = 'covariance';
info.MatPath{15} = 'initialization';
