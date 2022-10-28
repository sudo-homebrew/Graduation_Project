function [data, info] = gains
%Gains gives an empty data for r2_msgs/Gains

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'r2_msgs/Gains';
[data.JointNames, info.JointNames] = ros.internal.ros.messages.ros.char('string',NaN);
[data.K, info.K] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.D, info.D] = ros.internal.ros.messages.ros.default_type('double',NaN);
info.MessageType = 'r2_msgs/Gains';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'joint_names';
info.MatPath{2} = 'K';
info.MatPath{3} = 'D';
