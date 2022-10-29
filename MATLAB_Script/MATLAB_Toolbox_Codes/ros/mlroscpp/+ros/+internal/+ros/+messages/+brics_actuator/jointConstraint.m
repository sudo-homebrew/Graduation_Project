function [data, info] = jointConstraint
%JointConstraint gives an empty data for brics_actuator/JointConstraint

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'brics_actuator/JointConstraint';
[data.Type, info.Type] = ros.internal.ros.messages.ros.char('string',0);
[data.Value, info.Value] = ros.internal.ros.messages.brics_actuator.jointValue;
info.Value.MLdataType = 'struct';
info.MessageType = 'brics_actuator/JointConstraint';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'type';
info.MatPath{2} = 'value';
info.MatPath{3} = 'value.timeStamp';
info.MatPath{4} = 'value.timeStamp.sec';
info.MatPath{5} = 'value.timeStamp.nsec';
info.MatPath{6} = 'value.joint_uri';
info.MatPath{7} = 'value.unit';
info.MatPath{8} = 'value.value';
