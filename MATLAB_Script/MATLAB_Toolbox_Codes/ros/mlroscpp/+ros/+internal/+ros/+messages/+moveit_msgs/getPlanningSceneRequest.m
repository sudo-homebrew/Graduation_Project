function [data, info] = getPlanningSceneRequest
%GetPlanningScene gives an empty data for moveit_msgs/GetPlanningSceneRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/GetPlanningSceneRequest';
[data.Components, info.Components] = ros.internal.ros.messages.moveit_msgs.planningSceneComponents;
info.Components.MLdataType = 'struct';
info.MessageType = 'moveit_msgs/GetPlanningSceneRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'components';
info.MatPath{2} = 'components.SCENE_SETTINGS';
info.MatPath{3} = 'components.ROBOT_STATE';
info.MatPath{4} = 'components.ROBOT_STATE_ATTACHED_OBJECTS';
info.MatPath{5} = 'components.WORLD_OBJECT_NAMES';
info.MatPath{6} = 'components.WORLD_OBJECT_GEOMETRY';
info.MatPath{7} = 'components.OCTOMAP';
info.MatPath{8} = 'components.TRANSFORMS';
info.MatPath{9} = 'components.ALLOWED_COLLISION_MATRIX';
info.MatPath{10} = 'components.LINK_PADDING_AND_SCALING';
info.MatPath{11} = 'components.OBJECT_COLORS';
info.MatPath{12} = 'components.components';
