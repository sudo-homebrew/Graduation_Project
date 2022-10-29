function [data, info] = curve1D
%Curve1D gives an empty data for posedetection_msgs/Curve1D

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'posedetection_msgs/Curve1D';
[data.Pts, info.Pts] = ros.internal.ros.messages.ros.default_type('single',NaN);
info.MessageType = 'posedetection_msgs/Curve1D';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'pts';
