function [data, info] = setElevatorRequest
%SetElevator gives an empty data for robotnik_msgs/SetElevatorRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/SetElevatorRequest';
[data.Action, info.Action] = ros.internal.ros.messages.robotnik_msgs.elevatorAction;
info.Action.MLdataType = 'struct';
info.MessageType = 'robotnik_msgs/SetElevatorRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'action';
info.MatPath{2} = 'action.RAISE';
info.MatPath{3} = 'action.LOWER';
info.MatPath{4} = 'action.STOP';
info.MatPath{5} = 'action.NO_ACTION';
info.MatPath{6} = 'action.action';
