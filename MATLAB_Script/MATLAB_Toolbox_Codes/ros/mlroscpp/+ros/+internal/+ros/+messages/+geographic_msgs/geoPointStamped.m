function [data, info] = geoPointStamped
%GeoPointStamped gives an empty data for geographic_msgs/GeoPointStamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geographic_msgs/GeoPointStamped';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Position, info.Position] = ros.internal.ros.messages.geographic_msgs.geoPoint;
info.Position.MLdataType = 'struct';
info.MessageType = 'geographic_msgs/GeoPointStamped';
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
info.MatPath{7} = 'position';
info.MatPath{8} = 'position.latitude';
info.MatPath{9} = 'position.longitude';
info.MatPath{10} = 'position.altitude';