function [data, info] = controlResponse
%Control gives an empty data for grasp_stability_msgs/ControlResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'grasp_stability_msgs/ControlResponse';
[data.FAILURE, info.FAILURE] = ros.internal.ros.messages.ros.default_type('int32',1, 0);
[data.SUCCESS, info.SUCCESS] = ros.internal.ros.messages.ros.default_type('int32',1, 1);
[data.Result, info.Result] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'grasp_stability_msgs/ControlResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'FAILURE';
info.MatPath{2} = 'SUCCESS';
info.MatPath{3} = 'result';
