function [data, info] = powerBoardCommand2Response
%PowerBoardCommand2 gives an empty data for pr2_power_board/PowerBoardCommand2Response

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_power_board/PowerBoardCommand2Response';
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'pr2_power_board/PowerBoardCommand2Response';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'success';