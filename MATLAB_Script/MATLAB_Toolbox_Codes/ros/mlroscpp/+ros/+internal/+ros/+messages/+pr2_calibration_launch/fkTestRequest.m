function [data, info] = fkTestRequest
%FkTest gives an empty data for pr2_calibration_launch/FkTestRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_calibration_launch/FkTestRequest';
[data.Root, info.Root] = ros.internal.ros.messages.ros.char('string',0);
[data.Tip, info.Tip] = ros.internal.ros.messages.ros.char('string',0);
[data.JointPositions, info.JointPositions] = ros.internal.ros.messages.ros.default_type('double',NaN);
info.MessageType = 'pr2_calibration_launch/FkTestRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'root';
info.MatPath{2} = 'tip';
info.MatPath{3} = 'joint_positions';
