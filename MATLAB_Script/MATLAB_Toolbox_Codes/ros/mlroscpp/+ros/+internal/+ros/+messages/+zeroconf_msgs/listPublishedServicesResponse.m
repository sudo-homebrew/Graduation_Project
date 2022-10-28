function [data, info] = listPublishedServicesResponse
%ListPublishedServices gives an empty data for zeroconf_msgs/ListPublishedServicesResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'zeroconf_msgs/ListPublishedServicesResponse';
[data.Services, info.Services] = ros.internal.ros.messages.zeroconf_msgs.publishedService;
info.Services.MLdataType = 'struct';
info.Services.MaxLen = NaN;
info.Services.MinLen = 0;
data.Services = data.Services([],1);
info.MessageType = 'zeroconf_msgs/ListPublishedServicesResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'services';
info.MatPath{2} = 'services.name';
info.MatPath{3} = 'services.type';
info.MatPath{4} = 'services.domain';
info.MatPath{5} = 'services.port';
info.MatPath{6} = 'services.description';
