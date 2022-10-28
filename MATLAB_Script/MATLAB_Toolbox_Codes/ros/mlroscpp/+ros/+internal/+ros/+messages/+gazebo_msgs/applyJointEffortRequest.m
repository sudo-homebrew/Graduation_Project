function [data, info] = applyJointEffortRequest
%ApplyJointEffort gives an empty data for gazebo_msgs/ApplyJointEffortRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gazebo_msgs/ApplyJointEffortRequest';
[data.JointName, info.JointName] = ros.internal.ros.messages.ros.char('string',0);
[data.Effort, info.Effort] = ros.internal.ros.messages.ros.default_type('double',1);
[data.StartTime, info.StartTime] = ros.internal.ros.messages.ros.time;
info.StartTime.MLdataType = 'struct';
[data.Duration, info.Duration] = ros.internal.ros.messages.ros.duration;
info.Duration.MLdataType = 'struct';
info.MessageType = 'gazebo_msgs/ApplyJointEffortRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'joint_name';
info.MatPath{2} = 'effort';
info.MatPath{3} = 'start_time';
info.MatPath{4} = 'start_time.sec';
info.MatPath{5} = 'start_time.nsec';
info.MatPath{6} = 'duration';
info.MatPath{7} = 'duration.sec';
info.MatPath{8} = 'duration.nsec';
