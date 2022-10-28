function [data, info] = manipulationResult
%ManipulationResult gives an empty data for manipulation_msgs/ManipulationResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'manipulation_msgs/ManipulationResult';
[data.SUCCESS, info.SUCCESS] = ros.internal.ros.messages.ros.default_type('int32',1, 1);
[data.UNFEASIBLE, info.UNFEASIBLE] = ros.internal.ros.messages.ros.default_type('int32',1, -1);
[data.FAILED, info.FAILED] = ros.internal.ros.messages.ros.default_type('int32',1, -2);
[data.ERROR, info.ERROR] = ros.internal.ros.messages.ros.default_type('int32',1, -3);
[data.ARMMOVEMENTPREVENTED, info.ARMMOVEMENTPREVENTED] = ros.internal.ros.messages.ros.default_type('int32',1, -4);
[data.LIFTFAILED, info.LIFTFAILED] = ros.internal.ros.messages.ros.default_type('int32',1, -5);
[data.RETREATFAILED, info.RETREATFAILED] = ros.internal.ros.messages.ros.default_type('int32',1, -6);
[data.CANCELLED, info.CANCELLED] = ros.internal.ros.messages.ros.default_type('int32',1, -7);
[data.Value, info.Value] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'manipulation_msgs/ManipulationResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'SUCCESS';
info.MatPath{2} = 'UNFEASIBLE';
info.MatPath{3} = 'FAILED';
info.MatPath{4} = 'ERROR';
info.MatPath{5} = 'ARM_MOVEMENT_PREVENTED';
info.MatPath{6} = 'LIFT_FAILED';
info.MatPath{7} = 'RETREAT_FAILED';
info.MatPath{8} = 'CANCELLED';
info.MatPath{9} = 'value';
