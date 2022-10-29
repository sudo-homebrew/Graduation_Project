function [data, info] = objectRecognitionGoal
%ObjectRecognitionGoal gives an empty data for object_recognition_msgs/ObjectRecognitionGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'object_recognition_msgs/ObjectRecognitionGoal';
[data.UseRoi, info.UseRoi] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.FilterLimits, info.FilterLimits] = ros.internal.ros.messages.ros.default_type('single',NaN);
info.MessageType = 'object_recognition_msgs/ObjectRecognitionGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'use_roi';
info.MatPath{2} = 'filter_limits';
