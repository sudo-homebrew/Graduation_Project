function [data, info] = lookAtGoal
%LookAtGoal gives an empty data for cob_lookat_action/LookAtGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_lookat_action/LookAtGoal';
[data.TargetFrame, info.TargetFrame] = ros.internal.ros.messages.ros.char('string',0);
[data.PointingFrame, info.PointingFrame] = ros.internal.ros.messages.ros.char('string',0);
[data.XPOSITIVE, info.XPOSITIVE] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.YPOSITIVE, info.YPOSITIVE] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.ZPOSITIVE, info.ZPOSITIVE] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.XNEGATIVE, info.XNEGATIVE] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.YNEGATIVE, info.YNEGATIVE] = ros.internal.ros.messages.ros.default_type('uint8',1, 4);
[data.ZNEGATIVE, info.ZNEGATIVE] = ros.internal.ros.messages.ros.default_type('uint8',1, 5);
[data.PointingAxisType, info.PointingAxisType] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.BaseActive, info.BaseActive] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'cob_lookat_action/LookAtGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'target_frame';
info.MatPath{2} = 'pointing_frame';
info.MatPath{3} = 'X_POSITIVE';
info.MatPath{4} = 'Y_POSITIVE';
info.MatPath{5} = 'Z_POSITIVE';
info.MatPath{6} = 'X_NEGATIVE';
info.MatPath{7} = 'Y_NEGATIVE';
info.MatPath{8} = 'Z_NEGATIVE';
info.MatPath{9} = 'pointing_axis_type';
info.MatPath{10} = 'base_active';
