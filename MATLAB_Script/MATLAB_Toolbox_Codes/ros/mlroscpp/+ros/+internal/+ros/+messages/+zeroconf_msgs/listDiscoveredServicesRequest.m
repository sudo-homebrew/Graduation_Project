function [data, info] = listDiscoveredServicesRequest
%ListDiscoveredServices gives an empty data for zeroconf_msgs/ListDiscoveredServicesRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'zeroconf_msgs/ListDiscoveredServicesRequest';
[data.ServiceType, info.ServiceType] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'zeroconf_msgs/ListDiscoveredServicesRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'service_type';