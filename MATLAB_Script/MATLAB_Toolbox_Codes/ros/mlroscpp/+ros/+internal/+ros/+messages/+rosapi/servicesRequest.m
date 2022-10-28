function [data, info] = servicesRequest
%Services gives an empty data for rosapi/ServicesRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosapi/ServicesRequest';
info.MessageType = 'rosapi/ServicesRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
