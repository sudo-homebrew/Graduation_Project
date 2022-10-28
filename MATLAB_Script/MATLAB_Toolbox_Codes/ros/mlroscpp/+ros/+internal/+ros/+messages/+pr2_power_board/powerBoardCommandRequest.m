function [data, info] = powerBoardCommandRequest
%PowerBoardCommand gives an empty data for pr2_power_board/PowerBoardCommandRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_power_board/PowerBoardCommandRequest';
[data.SerialNumber, info.SerialNumber] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.BreakerNumber, info.BreakerNumber] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Command, info.Command] = ros.internal.ros.messages.ros.char('string',0);
[data.Flags, info.Flags] = ros.internal.ros.messages.ros.default_type('uint32',1);
info.MessageType = 'pr2_power_board/PowerBoardCommandRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'serial_number';
info.MatPath{2} = 'breaker_number';
info.MatPath{3} = 'command';
info.MatPath{4} = 'flags';
