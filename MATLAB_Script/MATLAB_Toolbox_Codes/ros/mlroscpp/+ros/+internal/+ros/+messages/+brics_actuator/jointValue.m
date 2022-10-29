function [data, info] = jointValue
%JointValue gives an empty data for brics_actuator/JointValue

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'brics_actuator/JointValue';
[data.TimeStamp, info.TimeStamp] = ros.internal.ros.messages.ros.time;
info.TimeStamp.MLdataType = 'struct';
[data.JointUri, info.JointUri] = ros.internal.ros.messages.ros.char('string',0);
[data.Unit, info.Unit] = ros.internal.ros.messages.ros.char('string',0);
[data.Value, info.Value] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'brics_actuator/JointValue';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'timeStamp';
info.MatPath{2} = 'timeStamp.sec';
info.MatPath{3} = 'timeStamp.nsec';
info.MatPath{4} = 'joint_uri';
info.MatPath{5} = 'unit';
info.MatPath{6} = 'value';
