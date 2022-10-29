function [data, info] = paramSetRequest
%ParamSet gives an empty data for mavros_msgs/ParamSetRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/ParamSetRequest';
[data.ParamId, info.ParamId] = ros.internal.ros.messages.ros.char('string',0);
[data.Value, info.Value] = ros.internal.ros.messages.mavros_msgs.paramValue;
info.Value.MLdataType = 'struct';
info.MessageType = 'mavros_msgs/ParamSetRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'param_id';
info.MatPath{2} = 'value';
info.MatPath{3} = 'value.integer';
info.MatPath{4} = 'value.real';
