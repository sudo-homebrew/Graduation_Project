function [data, info] = composeTrajectoryRequest
%ComposeTrajectory gives an empty data for cob_script_server/ComposeTrajectoryRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_script_server/ComposeTrajectoryRequest';
[data.ComponentName, info.ComponentName] = ros.internal.ros.messages.ros.char('string',0);
[data.ParameterName, info.ParameterName] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'cob_script_server/ComposeTrajectoryRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'component_name';
info.MatPath{2} = 'parameter_name';
