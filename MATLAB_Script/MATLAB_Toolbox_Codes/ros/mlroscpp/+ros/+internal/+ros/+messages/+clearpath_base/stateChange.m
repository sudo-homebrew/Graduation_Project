function [data, info] = stateChange
%StateChange gives an empty data for clearpath_base/StateChange

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'clearpath_base/StateChange';
[data.NewState, info.NewState] = ros.internal.ros.messages.ros.char('string',0);
[data.Joystick, info.Joystick] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'clearpath_base/StateChange';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'new_state';
info.MatPath{2} = 'joystick';
