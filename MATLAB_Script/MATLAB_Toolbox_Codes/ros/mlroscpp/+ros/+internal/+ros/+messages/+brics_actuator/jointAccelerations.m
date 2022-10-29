function [data, info] = jointAccelerations
%JointAccelerations gives an empty data for brics_actuator/JointAccelerations

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'brics_actuator/JointAccelerations';
[data.PoisonStamp, info.PoisonStamp] = ros.internal.ros.messages.brics_actuator.poison;
info.PoisonStamp.MLdataType = 'struct';
[data.Accelerations, info.Accelerations] = ros.internal.ros.messages.brics_actuator.jointValue;
info.Accelerations.MLdataType = 'struct';
info.Accelerations.MaxLen = NaN;
info.Accelerations.MinLen = 0;
data.Accelerations = data.Accelerations([],1);
info.MessageType = 'brics_actuator/JointAccelerations';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'poisonStamp';
info.MatPath{2} = 'poisonStamp.originator';
info.MatPath{3} = 'poisonStamp.description';
info.MatPath{4} = 'poisonStamp.qos';
info.MatPath{5} = 'accelerations';
info.MatPath{6} = 'accelerations.timeStamp';
info.MatPath{7} = 'accelerations.timeStamp.sec';
info.MatPath{8} = 'accelerations.timeStamp.nsec';
info.MatPath{9} = 'accelerations.joint_uri';
info.MatPath{10} = 'accelerations.unit';
info.MatPath{11} = 'accelerations.value';
