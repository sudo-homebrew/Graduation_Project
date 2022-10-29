function [data, info] = solvePositionIKResponse
%SolvePositionIK gives an empty data for baxter_core_msgs/SolvePositionIKResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'baxter_core_msgs/SolvePositionIKResponse';
[data.Joints, info.Joints] = ros.internal.ros.messages.sensor_msgs.jointState;
info.Joints.MLdataType = 'struct';
info.Joints.MaxLen = NaN;
info.Joints.MinLen = 0;
data.Joints = data.Joints([],1);
[data.IsValid, info.IsValid] = ros.internal.ros.messages.ros.default_type('logical',NaN);
[data.RESULTINVALID, info.RESULTINVALID] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.ResultType, info.ResultType] = ros.internal.ros.messages.ros.default_type('uint8',NaN);
info.MessageType = 'baxter_core_msgs/SolvePositionIKResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'joints';
info.MatPath{2} = 'joints.header';
info.MatPath{3} = 'joints.header.seq';
info.MatPath{4} = 'joints.header.stamp';
info.MatPath{5} = 'joints.header.stamp.sec';
info.MatPath{6} = 'joints.header.stamp.nsec';
info.MatPath{7} = 'joints.header.frame_id';
info.MatPath{8} = 'joints.name';
info.MatPath{9} = 'joints.position';
info.MatPath{10} = 'joints.velocity';
info.MatPath{11} = 'joints.effort';
info.MatPath{12} = 'isValid';
info.MatPath{13} = 'RESULT_INVALID';
info.MatPath{14} = 'result_type';
