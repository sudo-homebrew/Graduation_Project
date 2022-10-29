function [data, info] = configGoal
%ConfigGoal gives an empty data for joint_states_settler/ConfigGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'joint_states_settler/ConfigGoal';
[data.JointNames, info.JointNames] = ros.internal.ros.messages.ros.char('string',NaN);
[data.Tolerances, info.Tolerances] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.MaxStep, info.MaxStep] = ros.internal.ros.messages.ros.duration;
info.MaxStep.MLdataType = 'struct';
[data.CacheSize, info.CacheSize] = ros.internal.ros.messages.ros.default_type('uint32',1);
info.MessageType = 'joint_states_settler/ConfigGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'joint_names';
info.MatPath{2} = 'tolerances';
info.MatPath{3} = 'max_step';
info.MatPath{4} = 'max_step.sec';
info.MatPath{5} = 'max_step.nsec';
info.MatPath{6} = 'cache_size';
