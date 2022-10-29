function [data, info] = tareData
%TareData gives an empty data for baxter_maintenance_msgs/TareData

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'baxter_maintenance_msgs/TareData';
[data.TuneGravitySpring, info.TuneGravitySpring] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'baxter_maintenance_msgs/TareData';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'tuneGravitySpring';
