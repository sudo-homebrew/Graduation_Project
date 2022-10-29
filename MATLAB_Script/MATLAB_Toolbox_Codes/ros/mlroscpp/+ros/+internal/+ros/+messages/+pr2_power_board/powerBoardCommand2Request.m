function [data, info] = powerBoardCommand2Request
%PowerBoardCommand2 gives an empty data for pr2_power_board/PowerBoardCommand2Request

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_power_board/PowerBoardCommand2Request';
[data.NUMBEROFCIRCUITS, info.NUMBEROFCIRCUITS] = ros.internal.ros.messages.ros.default_type('int32',1, 3);
[data.Circuit, info.Circuit] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.BASE, info.BASE] = ros.internal.ros.messages.ros.default_type('int32',1, 0);
[data.RIGHTARM, info.RIGHTARM] = ros.internal.ros.messages.ros.default_type('int32',1, 1);
[data.LEFTARM, info.LEFTARM] = ros.internal.ros.messages.ros.default_type('int32',1, 2);
[data.Command, info.Command] = ros.internal.ros.messages.ros.char('string',0);
[data.ResetStats, info.ResetStats] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.ResetCircuits, info.ResetCircuits] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'pr2_power_board/PowerBoardCommand2Request';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'NUMBER_OF_CIRCUITS';
info.MatPath{2} = 'circuit';
info.MatPath{3} = 'BASE';
info.MatPath{4} = 'RIGHT_ARM';
info.MatPath{5} = 'LEFT_ARM';
info.MatPath{6} = 'command';
info.MatPath{7} = 'reset_stats';
info.MatPath{8} = 'reset_circuits';
