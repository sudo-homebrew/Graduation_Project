function [data, info] = constraintEvalResult
%ConstraintEvalResult gives an empty data for moveit_msgs/ConstraintEvalResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/ConstraintEvalResult';
[data.Result, info.Result] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Distance, info.Distance] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'moveit_msgs/ConstraintEvalResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'result';
info.MatPath{2} = 'distance';
