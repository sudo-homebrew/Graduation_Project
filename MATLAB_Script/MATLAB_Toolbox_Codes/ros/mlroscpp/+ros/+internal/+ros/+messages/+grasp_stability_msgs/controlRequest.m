function [data, info] = controlRequest
%Control gives an empty data for grasp_stability_msgs/ControlRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'grasp_stability_msgs/ControlRequest';
[data.CTRLSTART, info.CTRLSTART] = ros.internal.ros.messages.ros.default_type('int32',1, 0);
[data.CTRLSTOP, info.CTRLSTOP] = ros.internal.ros.messages.ros.default_type('int32',1, 1);
[data.Command, info.Command] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.MeasurementContextId, info.MeasurementContextId] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'grasp_stability_msgs/ControlRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'CTRL_START';
info.MatPath{2} = 'CTRL_STOP';
info.MatPath{3} = 'command';
info.MatPath{4} = 'measurement_context_id';
