function [data, info] = environmentLockResponse
%EnvironmentLock gives an empty data for jsk_pcl_ros/EnvironmentLockResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_pcl_ros/EnvironmentLockResponse';
[data.EnvironmentId, info.EnvironmentId] = ros.internal.ros.messages.ros.default_type('uint32',1);
info.MessageType = 'jsk_pcl_ros/EnvironmentLockResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'environment_id';
