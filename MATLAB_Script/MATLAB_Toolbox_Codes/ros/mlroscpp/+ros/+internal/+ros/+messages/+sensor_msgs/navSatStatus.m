function [data, info] = navSatStatus
%NavSatStatus gives an empty data for sensor_msgs/NavSatStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sensor_msgs/NavSatStatus';
[data.STATUSNOFIX, info.STATUSNOFIX] = ros.internal.ros.messages.ros.default_type('int8',1, -1);
[data.STATUSFIX, info.STATUSFIX] = ros.internal.ros.messages.ros.default_type('int8',1, 0);
[data.STATUSSBASFIX, info.STATUSSBASFIX] = ros.internal.ros.messages.ros.default_type('int8',1, 1);
[data.STATUSGBASFIX, info.STATUSGBASFIX] = ros.internal.ros.messages.ros.default_type('int8',1, 2);
[data.Status, info.Status] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.SERVICEGPS, info.SERVICEGPS] = ros.internal.ros.messages.ros.default_type('uint16',1, 1);
[data.SERVICEGLONASS, info.SERVICEGLONASS] = ros.internal.ros.messages.ros.default_type('uint16',1, 2);
[data.SERVICECOMPASS, info.SERVICECOMPASS] = ros.internal.ros.messages.ros.default_type('uint16',1, 4);
[data.SERVICEGALILEO, info.SERVICEGALILEO] = ros.internal.ros.messages.ros.default_type('uint16',1, 8);
[data.Service, info.Service] = ros.internal.ros.messages.ros.default_type('uint16',1);
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
