function [data, info] = navSatStatus
%NavSatStatus gives an empty data for sensor_msgs/NavSatStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sensor_msgs/NavSatStatus';
[data.STATUS_NO_FIX, info.STATUS_NO_FIX] = ros.internal.ros2.messages.ros2.default_type('int8',1,0, -1, [NaN]);
[data.STATUS_FIX, info.STATUS_FIX] = ros.internal.ros2.messages.ros2.default_type('int8',1,0, 0, [NaN]);
[data.STATUS_SBAS_FIX, info.STATUS_SBAS_FIX] = ros.internal.ros2.messages.ros2.default_type('int8',1,0, 1, [NaN]);
[data.STATUS_GBAS_FIX, info.STATUS_GBAS_FIX] = ros.internal.ros2.messages.ros2.default_type('int8',1,0, 2, [NaN]);
[data.status, info.status] = ros.internal.ros2.messages.ros2.default_type('int8',1,0);
[data.SERVICE_GPS, info.SERVICE_GPS] = ros.internal.ros2.messages.ros2.default_type('uint16',1,0, 1, [NaN]);
[data.SERVICE_GLONASS, info.SERVICE_GLONASS] = ros.internal.ros2.messages.ros2.default_type('uint16',1,0, 2, [NaN]);
[data.SERVICE_COMPASS, info.SERVICE_COMPASS] = ros.internal.ros2.messages.ros2.default_type('uint16',1,0, 4, [NaN]);
[data.SERVICE_GALILEO, info.SERVICE_GALILEO] = ros.internal.ros2.messages.ros2.default_type('uint16',1,0, 8, [NaN]);
[data.service, info.service] = ros.internal.ros2.messages.ros2.default_type('uint16',1,0);
info.MessageType = 'sensor_msgs/NavSatStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'STATUS_NO_FIX';
info.MatPath{2} = 'STATUS_FIX';
info.MatPath{3} = 'STATUS_SBAS_FIX';
info.MatPath{4} = 'STATUS_GBAS_FIX';
info.MatPath{5} = 'status';
info.MatPath{6} = 'SERVICE_GPS';
info.MatPath{7} = 'SERVICE_GLONASS';
info.MatPath{8} = 'SERVICE_COMPASS';
info.MatPath{9} = 'SERVICE_GALILEO';
info.MatPath{10} = 'service';
