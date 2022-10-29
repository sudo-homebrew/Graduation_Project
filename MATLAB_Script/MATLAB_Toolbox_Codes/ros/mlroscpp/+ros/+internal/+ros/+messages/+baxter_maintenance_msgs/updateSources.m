function [data, info] = updateSources
%UpdateSources gives an empty data for baxter_maintenance_msgs/UpdateSources

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'baxter_maintenance_msgs/UpdateSources';
[data.Uuid, info.Uuid] = ros.internal.ros.messages.ros.char('string',0);
[data.Sources, info.Sources] = ros.internal.ros.messages.baxter_maintenance_msgs.updateSource;
info.Sources.MLdataType = 'struct';
info.Sources.MaxLen = NaN;
info.Sources.MinLen = 0;
data.Sources = data.Sources([],1);
info.MessageType = 'baxter_maintenance_msgs/UpdateSources';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'uuid';
info.MatPath{2} = 'sources';
info.MatPath{3} = 'sources.devname';
info.MatPath{4} = 'sources.filename';
info.MatPath{5} = 'sources.version';
info.MatPath{6} = 'sources.uuid';
