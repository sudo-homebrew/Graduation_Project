function [data, info] = rOSMaster
%ROSMaster gives an empty data for multimaster_msgs_fkie/ROSMaster

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'multimaster_msgs_fkie/ROSMaster';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Uri, info.Uri] = ros.internal.ros.messages.ros.char('string',0);
[data.Timestamp, info.Timestamp] = ros.internal.ros.messages.ros.default_type('double',1);
[data.TimestampLocal, info.TimestampLocal] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Online, info.Online] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.DiscovererName, info.DiscovererName] = ros.internal.ros.messages.ros.char('string',0);
[data.Monitoruri, info.Monitoruri] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'multimaster_msgs_fkie/ROSMaster';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'name';
info.MatPath{2} = 'uri';
info.MatPath{3} = 'timestamp';
info.MatPath{4} = 'timestamp_local';
info.MatPath{5} = 'online';
info.MatPath{6} = 'discoverer_name';
info.MatPath{7} = 'monitoruri';
