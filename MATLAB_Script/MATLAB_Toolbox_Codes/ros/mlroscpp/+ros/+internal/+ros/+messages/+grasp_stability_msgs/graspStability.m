function [data, info] = graspStability
%GraspStability gives an empty data for grasp_stability_msgs/GraspStability

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'grasp_stability_msgs/GraspStability';
[data.MeasurementContextId, info.MeasurementContextId] = ros.internal.ros.messages.ros.char('string',0);
[data.GraspQuality, info.GraspQuality] = ros.internal.ros.messages.ros.default_type('single',1);
[data.EstimationConfidence, info.EstimationConfidence] = ros.internal.ros.messages.ros.default_type('single',1);
[data.GRASPCATUNDEFINED, info.GRASPCATUNDEFINED] = ros.internal.ros.messages.ros.default_type('int32',1, 0);
[data.GRASPCATGOOD, info.GRASPCATGOOD] = ros.internal.ros.messages.ros.default_type('int32',1, 1);
[data.GRASPCATMEDIUM, info.GRASPCATMEDIUM] = ros.internal.ros.messages.ros.default_type('int32',1, 2);
[data.GRASPCATBAD, info.GRASPCATBAD] = ros.internal.ros.messages.ros.default_type('int32',1, 3);
[data.GraspCategory, info.GraspCategory] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'grasp_stability_msgs/GraspStability';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'measurement_context_id';
info.MatPath{2} = 'grasp_quality';
info.MatPath{3} = 'estimation_confidence';
info.MatPath{4} = 'GRASP_CAT_UNDEFINED';
info.MatPath{5} = 'GRASP_CAT_GOOD';
info.MatPath{6} = 'GRASP_CAT_MEDIUM';
info.MatPath{7} = 'GRASP_CAT_BAD';
info.MatPath{8} = 'grasp_category';
