function [data, info] = setElevatorResult
%SetElevatorResult gives an empty data for robotnik_msgs/SetElevatorResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/SetElevatorResult';
[data.Result, info.Result] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Status, info.Status] = ros.internal.ros.messages.robotnik_msgs.elevatorStatus;
info.Status.MLdataType = 'struct';
info.MessageType = 'robotnik_msgs/SetElevatorResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'result';
info.MatPath{2} = 'status';
info.MatPath{3} = 'status.RAISING';
info.MatPath{4} = 'status.LOWERING';
info.MatPath{5} = 'status.IDLE';
info.MatPath{6} = 'status.ERROR_G_IO';
info.MatPath{7} = 'status.ERROR_S_IO';
info.MatPath{8} = 'status.ERROR_TIMEOUT';
info.MatPath{9} = 'status.UP';
info.MatPath{10} = 'status.DOWN';
info.MatPath{11} = 'status.UNKNOWN';
info.MatPath{12} = 'status.state';
info.MatPath{13} = 'status.position';
info.MatPath{14} = 'status.height';
