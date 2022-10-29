function [data, info] = publishedService
%PublishedService gives an empty data for zeroconf_msgs/PublishedService

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'zeroconf_msgs/PublishedService';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Type, info.Type] = ros.internal.ros.messages.ros.char('string',0);
[data.Domain, info.Domain] = ros.internal.ros.messages.ros.char('string',0);
[data.Port, info.Port] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Description, info.Description] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'zeroconf_msgs/PublishedService';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'name';
info.MatPath{2} = 'type';
info.MatPath{3} = 'domain';
info.MatPath{4} = 'port';
info.MatPath{5} = 'description';
