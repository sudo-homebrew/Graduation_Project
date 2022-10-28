function [data, info] = updateSource
%UpdateSource gives an empty data for baxter_maintenance_msgs/UpdateSource

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'baxter_maintenance_msgs/UpdateSource';
[data.Devname, info.Devname] = ros.internal.ros.messages.ros.char('string',0);
[data.Filename, info.Filename] = ros.internal.ros.messages.ros.char('string',0);
[data.Version, info.Version] = ros.internal.ros.messages.ros.char('string',0);
[data.Uuid, info.Uuid] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'baxter_maintenance_msgs/UpdateSource';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'devname';
info.MatPath{2} = 'filename';
info.MatPath{3} = 'version';
info.MatPath{4} = 'uuid';
