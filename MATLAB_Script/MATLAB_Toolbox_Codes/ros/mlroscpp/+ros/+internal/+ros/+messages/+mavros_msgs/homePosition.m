function [data, info] = homePosition
%HomePosition gives an empty data for mavros_msgs/HomePosition

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/HomePosition';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Geo, info.Geo] = ros.internal.ros.messages.geographic_msgs.geoPoint;
info.Geo.MLdataType = 'struct';
[data.Position, info.Position] = ros.internal.ros.messages.geometry_msgs.point;
info.Position.MLdataType = 'struct';
[data.Orientation, info.Orientation] = ros.internal.ros.messages.geometry_msgs.quaternion;
info.Orientation.MLdataType = 'struct';
[data.Approach, info.Approach] = ros.internal.ros.messages.geometry_msgs.vector3;
info.Approach.MLdataType = 'struct';
info.MessageType = 'mavros_msgs/HomePosition';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,23);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'geo';
info.MatPath{8} = 'geo.latitude';
info.MatPath{9} = 'geo.longitude';
info.MatPath{10} = 'geo.altitude';
info.MatPath{11} = 'position';
info.MatPath{12} = 'position.x';
info.MatPath{13} = 'position.y';
info.MatPath{14} = 'position.z';
info.MatPath{15} = 'orientation';
info.MatPath{16} = 'orientation.x';
info.MatPath{17} = 'orientation.y';
info.MatPath{18} = 'orientation.z';
info.MatPath{19} = 'orientation.w';
info.MatPath{20} = 'approach';
info.MatPath{21} = 'approach.x';
info.MatPath{22} = 'approach.y';
info.MatPath{23} = 'approach.z';