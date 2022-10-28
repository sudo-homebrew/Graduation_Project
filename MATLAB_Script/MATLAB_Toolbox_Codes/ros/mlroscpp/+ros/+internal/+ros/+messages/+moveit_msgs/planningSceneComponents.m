function [data, info] = planningSceneComponents
%PlanningSceneComponents gives an empty data for moveit_msgs/PlanningSceneComponents

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/PlanningSceneComponents';
[data.SCENESETTINGS, info.SCENESETTINGS] = ros.internal.ros.messages.ros.default_type('uint32',1, 1);
[data.ROBOTSTATE, info.ROBOTSTATE] = ros.internal.ros.messages.ros.default_type('uint32',1, 2);
[data.ROBOTSTATEATTACHEDOBJECTS, info.ROBOTSTATEATTACHEDOBJECTS] = ros.internal.ros.messages.ros.default_type('uint32',1, 4);
[data.WORLDOBJECTNAMES, info.WORLDOBJECTNAMES] = ros.internal.ros.messages.ros.default_type('uint32',1, 8);
[data.WORLDOBJECTGEOMETRY, info.WORLDOBJECTGEOMETRY] = ros.internal.ros.messages.ros.default_type('uint32',1, 16);
[data.OCTOMAP, info.OCTOMAP] = ros.internal.ros.messages.ros.default_type('uint32',1, 32);
[data.TRANSFORMS, info.TRANSFORMS] = ros.internal.ros.messages.ros.default_type('uint32',1, 64);
[data.ALLOWEDCOLLISIONMATRIX, info.ALLOWEDCOLLISIONMATRIX] = ros.internal.ros.messages.ros.default_type('uint32',1, 128);
[data.LINKPADDINGANDSCALING, info.LINKPADDINGANDSCALING] = ros.internal.ros.messages.ros.default_type('uint32',1, 256);
[data.OBJECTCOLORS, info.OBJECTCOLORS] = ros.internal.ros.messages.ros.default_type('uint32',1, 512);
[data.Components, info.Components] = ros.internal.ros.messages.ros.default_type('uint32',1);
info.MessageType = 'moveit_msgs/PlanningSceneComponents';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'SCENE_SETTINGS';
info.MatPath{2} = 'ROBOT_STATE';
info.MatPath{3} = 'ROBOT_STATE_ATTACHED_OBJECTS';
info.MatPath{4} = 'WORLD_OBJECT_NAMES';
info.MatPath{5} = 'WORLD_OBJECT_GEOMETRY';
info.MatPath{6} = 'OCTOMAP';
info.MatPath{7} = 'TRANSFORMS';
info.MatPath{8} = 'ALLOWED_COLLISION_MATRIX';
info.MatPath{9} = 'LINK_PADDING_AND_SCALING';
info.MatPath{10} = 'OBJECT_COLORS';
info.MatPath{11} = 'components';
