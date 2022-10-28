function [data, info] = poseRT
%PoseRT gives an empty data for cob_object_detection_msgs/PoseRT

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_object_detection_msgs/PoseRT';
[data.Rvec, info.Rvec] = ros.internal.ros.messages.ros.default_type('double',3);
[data.Tvec, info.Tvec] = ros.internal.ros.messages.ros.default_type('double',3);
info.MessageType = 'cob_object_detection_msgs/PoseRT';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'rvec';
info.MatPath{2} = 'tvec';
