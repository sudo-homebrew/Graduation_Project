function [data, info] = jointTorques
%JointTorques gives an empty data for brics_actuator/JointTorques

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'brics_actuator/JointTorques';
[data.PoisonStamp, info.PoisonStamp] = ros.internal.ros.messages.brics_actuator.poison;
info.PoisonStamp.MLdataType = 'struct';
[data.Torques, info.Torques] = ros.internal.ros.messages.brics_actuator.jointValue;
info.Torques.MLdataType = 'struct';
info.Torques.MaxLen = NaN;
info.Torques.MinLen = 0;
data.Torques = data.Torques([],1);
info.MessageType = 'brics_actuator/JointTorques';
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
info.MatPath{5} = 'torques';
info.MatPath{6} = 'torques.timeStamp';
info.MatPath{7} = 'torques.timeStamp.sec';
info.MatPath{8} = 'torques.timeStamp.nsec';
info.MatPath{9} = 'torques.joint_uri';
info.MatPath{10} = 'torques.unit';
info.MatPath{11} = 'torques.value';
