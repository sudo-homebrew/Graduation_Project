function [data, info] = geoPoint
%GeoPoint gives an empty data for geographic_msgs/GeoPoint

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geographic_msgs/GeoPoint';
[data.Latitude, info.Latitude] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Longitude, info.Longitude] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Altitude, info.Altitude] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'geographic_msgs/GeoPoint';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'latitude';
info.MatPath{2} = 'longitude';
info.MatPath{3} = 'altitude';
