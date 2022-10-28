function [data, info] = paramSetResponse
%ParamSet gives an empty data for mavros_msgs/ParamSetResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/ParamSetResponse';
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Value, info.Value] = ros.internal.ros.messages.mavros_msgs.paramValue;
info.Value.MLdataType = 'struct';
info.MessageType = 'mavros_msgs/ParamSetResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'success';
info.MatPath{2} = 'value';
info.MatPath{3} = 'value.integer';
info.MatPath{4} = 'value.real';
