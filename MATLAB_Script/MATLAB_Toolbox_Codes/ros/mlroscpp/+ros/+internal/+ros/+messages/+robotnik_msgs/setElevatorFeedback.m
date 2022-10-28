function [data, info] = setElevatorFeedback
%SetElevatorFeedback gives an empty data for robotnik_msgs/SetElevatorFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/SetElevatorFeedback';
[data.Status, info.Status] = ros.internal.ros.messages.robotnik_msgs.elevatorStatus;
info.Status.MLdataType = 'struct';
info.MessageType = 'robotnik_msgs/SetElevatorFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,13);
info.MatPath{1} = 'status';
info.MatPath{2} = 'status.RAISING';
info.MatPath{3} = 'status.LOWERING';
info.MatPath{4} = 'status.IDLE';
info.MatPath{5} = 'status.ERROR_G_IO';
info.MatPath{6} = 'status.ERROR_S_IO';
info.MatPath{7} = 'status.ERROR_TIMEOUT';
info.MatPath{8} = 'status.UP';
info.MatPath{9} = 'status.DOWN';
info.MatPath{10} = 'status.UNKNOWN';
info.MatPath{11} = 'status.state';
info.MatPath{12} = 'status.position';
info.MatPath{13} = 'status.height';
