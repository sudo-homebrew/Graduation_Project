function [data, info] = laserStatus
%LaserStatus gives an empty data for robotnik_msgs/LaserStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/LaserStatus';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.DetectingObstacles, info.DetectingObstacles] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Contaminated, info.Contaminated] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.FreeWarning, info.FreeWarning] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.WarningZones, info.WarningZones] = ros.internal.ros.messages.ros.default_type('logical',NaN);
info.MessageType = 'robotnik_msgs/LaserStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'name';
info.MatPath{2} = 'detecting_obstacles';
info.MatPath{3} = 'contaminated';
info.MatPath{4} = 'free_warning';
info.MatPath{5} = 'warning_zones';
