function [data, info] = powerBoardCommandResponse
%PowerBoardCommand gives an empty data for pr2_power_board/PowerBoardCommandResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_power_board/PowerBoardCommandResponse';
[data.Retval, info.Retval] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'pr2_power_board/PowerBoardCommandResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'retval';
