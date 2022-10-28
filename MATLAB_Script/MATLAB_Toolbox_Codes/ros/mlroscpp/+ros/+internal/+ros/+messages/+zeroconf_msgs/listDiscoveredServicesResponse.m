function [data, info] = listDiscoveredServicesResponse
%ListDiscoveredServices gives an empty data for zeroconf_msgs/ListDiscoveredServicesResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'zeroconf_msgs/ListDiscoveredServicesResponse';
[data.Services, info.Services] = ros.internal.ros.messages.zeroconf_msgs.discoveredService;
info.Services.MLdataType = 'struct';
info.Services.MaxLen = NaN;
info.Services.MinLen = 0;
data.Services = data.Services([],1);
info.MessageType = 'zeroconf_msgs/ListDiscoveredServicesResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,15);
info.MatPath{1} = 'services';
info.MatPath{2} = 'services.name';
info.MatPath{3} = 'services.type';
info.MatPath{4} = 'services.domain';
info.MatPath{5} = 'services.description';
info.MatPath{6} = 'services.hostname';
info.MatPath{7} = 'services.ipv4_addresses';
info.MatPath{8} = 'services.ipv6_addresses';
info.MatPath{9} = 'services.port';
info.MatPath{10} = 'services.cookie';
info.MatPath{11} = 'services.is_local';
info.MatPath{12} = 'services.our_own';
info.MatPath{13} = 'services.wide_area';
info.MatPath{14} = 'services.multicast';
info.MatPath{15} = 'services.cached';
