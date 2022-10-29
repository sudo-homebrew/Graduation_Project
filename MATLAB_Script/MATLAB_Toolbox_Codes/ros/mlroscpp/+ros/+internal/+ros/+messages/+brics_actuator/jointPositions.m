function [data, info] = jointPositions
%JointPositions gives an empty data for brics_actuator/JointPositions

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'brics_actuator/JointPositions';
[data.PoisonStamp, info.PoisonStamp] = ros.internal.ros.messages.brics_actuator.poison;
info.PoisonStamp.MLdataType = 'struct';
[data.Positions, info.Positions] = ros.internal.ros.messages.brics_actuator.jointValue;
info.Positions.MLdataType = 'struct';
info.Positions.MaxLen = NaN;
info.Positions.MinLen = 0;
data.Positions = data.Positions([],1);
info.MessageType = 'brics_actuator/JointPositions';
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
info.MatPath{5} = 'positions';
info.MatPath{6} = 'positions.timeStamp';
info.MatPath{7} = 'positions.timeStamp.sec';
info.MatPath{8} = 'positions.timeStamp.nsec';
info.MatPath{9} = 'positions.joint_uri';
info.MatPath{10} = 'positions.unit';
info.MatPath{11} = 'positions.value';
