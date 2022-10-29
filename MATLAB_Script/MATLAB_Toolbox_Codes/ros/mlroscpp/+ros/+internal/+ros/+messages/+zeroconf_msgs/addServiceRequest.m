function [data, info] = addServiceRequest
%AddService gives an empty data for zeroconf_msgs/AddServiceRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'zeroconf_msgs/AddServiceRequest';
[data.Service, info.Service] = ros.internal.ros.messages.zeroconf_msgs.publishedService;
info.Service.MLdataType = 'struct';
info.MessageType = 'zeroconf_msgs/AddServiceRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'service';
info.MatPath{2} = 'service.name';
info.MatPath{3} = 'service.type';
info.MatPath{4} = 'service.domain';
info.MatPath{5} = 'service.port';
info.MatPath{6} = 'service.description';
