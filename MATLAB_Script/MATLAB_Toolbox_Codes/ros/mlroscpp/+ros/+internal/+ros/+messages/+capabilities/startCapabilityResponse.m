function [data, info] = startCapabilityResponse
%StartCapability gives an empty data for capabilities/StartCapabilityResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'capabilities/StartCapabilityResponse';
[data.Result, info.Result] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.RESULTSUCCESS, info.RESULTSUCCESS] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.RESULTCURRENTLYSTARTING, info.RESULTCURRENTLYSTARTING] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.RESULTCURRENTLYRUNNING, info.RESULTCURRENTLYRUNNING] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.RESULTCURRENTLYSTOPPING, info.RESULTCURRENTLYSTOPPING] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
info.MessageType = 'capabilities/StartCapabilityResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'result';
info.MatPath{2} = 'RESULT_SUCCESS';
info.MatPath{3} = 'RESULT_CURRENTLY_STARTING';
info.MatPath{4} = 'RESULT_CURRENTLY_RUNNING';
info.MatPath{5} = 'RESULT_CURRENTLY_STOPPING';
