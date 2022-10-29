function [data, info] = userAccuracySpecs
%UserAccuracySpecs gives an empty data for applanix_msgs/UserAccuracySpecs

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/UserAccuracySpecs';
[data.Transaction, info.Transaction] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.UserAltitudeAccuracy, info.UserAltitudeAccuracy] = ros.internal.ros.messages.ros.default_type('single',1);
[data.UserHeadingAccuracy, info.UserHeadingAccuracy] = ros.internal.ros.messages.ros.default_type('single',1);
[data.UserPositionAccuracy, info.UserPositionAccuracy] = ros.internal.ros.messages.ros.default_type('single',1);
[data.UserVelocityAccuracy, info.UserVelocityAccuracy] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'applanix_msgs/UserAccuracySpecs';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'transaction';
info.MatPath{2} = 'user_altitude_accuracy';
info.MatPath{3} = 'user_heading_accuracy';
info.MatPath{4} = 'user_position_accuracy';
info.MatPath{5} = 'user_velocity_accuracy';
