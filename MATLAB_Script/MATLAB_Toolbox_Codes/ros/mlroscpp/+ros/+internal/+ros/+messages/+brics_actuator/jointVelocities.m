function [data, info] = jointVelocities
%JointVelocities gives an empty data for brics_actuator/JointVelocities

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'brics_actuator/JointVelocities';
[data.PoisonStamp, info.PoisonStamp] = ros.internal.ros.messages.brics_actuator.poison;
info.PoisonStamp.MLdataType = 'struct';
[data.Velocities, info.Velocities] = ros.internal.ros.messages.brics_actuator.jointValue;
info.Velocities.MLdataType = 'struct';
info.Velocities.MaxLen = NaN;
info.Velocities.MinLen = 0;
data.Velocities = data.Velocities([],1);
info.MessageType = 'brics_actuator/JointVelocities';
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
info.MatPath{5} = 'velocities';
info.MatPath{6} = 'velocities.timeStamp';
info.MatPath{7} = 'velocities.timeStamp.sec';
info.MatPath{8} = 'velocities.timeStamp.nsec';
info.MatPath{9} = 'velocities.joint_uri';
info.MatPath{10} = 'velocities.unit';
info.MatPath{11} = 'velocities.value';
