function [data, info] = graspPlanningErrorCode
%GraspPlanningErrorCode gives an empty data for manipulation_msgs/GraspPlanningErrorCode

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'manipulation_msgs/GraspPlanningErrorCode';
[data.SUCCESS, info.SUCCESS] = ros.internal.ros.messages.ros.default_type('int32',1, 0);
[data.TFERROR, info.TFERROR] = ros.internal.ros.messages.ros.default_type('int32',1, 1);
[data.OTHERERROR, info.OTHERERROR] = ros.internal.ros.messages.ros.default_type('int32',1, 2);
[data.Value, info.Value] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'manipulation_msgs/GraspPlanningErrorCode';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'SUCCESS';
info.MatPath{2} = 'TF_ERROR';
info.MatPath{3} = 'OTHER_ERROR';
info.MatPath{4} = 'value';
