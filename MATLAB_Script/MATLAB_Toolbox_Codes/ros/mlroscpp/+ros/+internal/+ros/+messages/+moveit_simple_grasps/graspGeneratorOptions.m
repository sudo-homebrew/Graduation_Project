function [data, info] = graspGeneratorOptions
%GraspGeneratorOptions gives an empty data for moveit_simple_grasps/GraspGeneratorOptions

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_simple_grasps/GraspGeneratorOptions';
[data.GraspAxis, info.GraspAxis] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.GRASPAXISX, info.GRASPAXISX] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.GRASPAXISY, info.GRASPAXISY] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.GRASPAXISZ, info.GRASPAXISZ] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.GraspDirection, info.GraspDirection] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.GRASPDIRECTIONUP, info.GRASPDIRECTIONUP] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.GRASPDIRECTIONDOWN, info.GRASPDIRECTIONDOWN] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.GraspRotation, info.GraspRotation] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.GRASPROTATIONHALF, info.GRASPROTATIONHALF] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.GRASPROTATIONFULL, info.GRASPROTATIONFULL] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
info.MessageType = 'moveit_simple_grasps/GraspGeneratorOptions';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'grasp_axis';
info.MatPath{2} = 'GRASP_AXIS_X';
info.MatPath{3} = 'GRASP_AXIS_Y';
info.MatPath{4} = 'GRASP_AXIS_Z';
info.MatPath{5} = 'grasp_direction';
info.MatPath{6} = 'GRASP_DIRECTION_UP';
info.MatPath{7} = 'GRASP_DIRECTION_DOWN';
info.MatPath{8} = 'grasp_rotation';
info.MatPath{9} = 'GRASP_ROTATION_HALF';
info.MatPath{10} = 'GRASP_ROTATION_FULL';
