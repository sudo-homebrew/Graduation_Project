function [data, info] = pDMCStatus
%PDMCStatus gives an empty data for r2_msgs/PDMCStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'r2_msgs/PDMCStatus';
[data.Publisher, info.Publisher] = ros.internal.ros.messages.ros.char('string',0);
[data.RegisterValue, info.RegisterValue] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.LogicEnabled, info.LogicEnabled] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.MotorEnabled, info.MotorEnabled] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'r2_msgs/PDMCStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'publisher';
info.MatPath{2} = 'registerValue';
info.MatPath{3} = 'logicEnabled';
info.MatPath{4} = 'motorEnabled';
