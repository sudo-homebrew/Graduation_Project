function [data, info] = followJointTrajectoryResult
%FollowJointTrajectoryResult gives an empty data for control_msgs/FollowJointTrajectoryResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'control_msgs/FollowJointTrajectoryResult';
[data.ErrorCode, info.ErrorCode] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.SUCCESSFUL, info.SUCCESSFUL] = ros.internal.ros.messages.ros.default_type('int32',1, 0);
[data.INVALIDGOAL, info.INVALIDGOAL] = ros.internal.ros.messages.ros.default_type('int32',1, -1);
[data.INVALIDJOINTS, info.INVALIDJOINTS] = ros.internal.ros.messages.ros.default_type('int32',1, -2);
[data.OLDHEADERTIMESTAMP, info.OLDHEADERTIMESTAMP] = ros.internal.ros.messages.ros.default_type('int32',1, -3);
[data.PATHTOLERANCEVIOLATED, info.PATHTOLERANCEVIOLATED] = ros.internal.ros.messages.ros.default_type('int32',1, -4);
[data.GOALTOLERANCEVIOLATED, info.GOALTOLERANCEVIOLATED] = ros.internal.ros.messages.ros.default_type('int32',1, -5);
[data.ErrorString, info.ErrorString] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'control_msgs/FollowJointTrajectoryResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'error_code';
info.MatPath{2} = 'SUCCESSFUL';
info.MatPath{3} = 'INVALID_GOAL';
info.MatPath{4} = 'INVALID_JOINTS';
info.MatPath{5} = 'OLD_HEADER_TIMESTAMP';
info.MatPath{6} = 'PATH_TOLERANCE_VIOLATED';
info.MatPath{7} = 'GOAL_TOLERANCE_VIOLATED';
info.MatPath{8} = 'error_string';
