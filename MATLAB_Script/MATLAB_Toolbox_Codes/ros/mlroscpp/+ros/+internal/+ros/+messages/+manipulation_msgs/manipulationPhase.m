function [data, info] = manipulationPhase
%ManipulationPhase gives an empty data for manipulation_msgs/ManipulationPhase

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'manipulation_msgs/ManipulationPhase';
[data.CHECKINGFEASIBILITY, info.CHECKINGFEASIBILITY] = ros.internal.ros.messages.ros.default_type('int32',1, 0);
[data.MOVINGTOPREGRASP, info.MOVINGTOPREGRASP] = ros.internal.ros.messages.ros.default_type('int32',1, 1);
[data.MOVINGTOGRASP, info.MOVINGTOGRASP] = ros.internal.ros.messages.ros.default_type('int32',1, 2);
[data.CLOSING, info.CLOSING] = ros.internal.ros.messages.ros.default_type('int32',1, 3);
[data.ADJUSTINGGRASP, info.ADJUSTINGGRASP] = ros.internal.ros.messages.ros.default_type('int32',1, 4);
[data.LIFTING, info.LIFTING] = ros.internal.ros.messages.ros.default_type('int32',1, 5);
[data.MOVINGWITHOBJECT, info.MOVINGWITHOBJECT] = ros.internal.ros.messages.ros.default_type('int32',1, 6);
[data.MOVINGTOPLACE, info.MOVINGTOPLACE] = ros.internal.ros.messages.ros.default_type('int32',1, 7);
[data.PLACING, info.PLACING] = ros.internal.ros.messages.ros.default_type('int32',1, 8);
[data.OPENING, info.OPENING] = ros.internal.ros.messages.ros.default_type('int32',1, 9);
[data.RETREATING, info.RETREATING] = ros.internal.ros.messages.ros.default_type('int32',1, 10);
[data.MOVINGWITHOUTOBJECT, info.MOVINGWITHOUTOBJECT] = ros.internal.ros.messages.ros.default_type('int32',1, 11);
[data.SHAKING, info.SHAKING] = ros.internal.ros.messages.ros.default_type('int32',1, 12);
[data.SUCCEEDED, info.SUCCEEDED] = ros.internal.ros.messages.ros.default_type('int32',1, 13);
[data.FAILED, info.FAILED] = ros.internal.ros.messages.ros.default_type('int32',1, 14);
[data.ABORTED, info.ABORTED] = ros.internal.ros.messages.ros.default_type('int32',1, 15);
[data.HOLDINGOBJECT, info.HOLDINGOBJECT] = ros.internal.ros.messages.ros.default_type('int32',1, 16);
[data.Phase, info.Phase] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'manipulation_msgs/ManipulationPhase';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,18);
info.MatPath{1} = 'CHECKING_FEASIBILITY';
info.MatPath{2} = 'MOVING_TO_PREGRASP';
info.MatPath{3} = 'MOVING_TO_GRASP';
info.MatPath{4} = 'CLOSING';
info.MatPath{5} = 'ADJUSTING_GRASP';
info.MatPath{6} = 'LIFTING';
info.MatPath{7} = 'MOVING_WITH_OBJECT';
info.MatPath{8} = 'MOVING_TO_PLACE';
info.MatPath{9} = 'PLACING';
info.MatPath{10} = 'OPENING';
info.MatPath{11} = 'RETREATING';
info.MatPath{12} = 'MOVING_WITHOUT_OBJECT';
info.MatPath{13} = 'SHAKING';
info.MatPath{14} = 'SUCCEEDED';
info.MatPath{15} = 'FAILED';
info.MatPath{16} = 'ABORTED';
info.MatPath{17} = 'HOLDING_OBJECT';
info.MatPath{18} = 'phase';