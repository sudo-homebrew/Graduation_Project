function [data, info] = digitalIOState
%DigitalIOState gives an empty data for baxter_core_msgs/DigitalIOState

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'baxter_core_msgs/DigitalIOState';
[data.State, info.State] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.IsInputOnly, info.IsInputOnly] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.OFF, info.OFF] = ros.internal.ros.messages.ros.default_type('int8',1, 0);
[data.ON, info.ON] = ros.internal.ros.messages.ros.default_type('int8',1, 1);
[data.PRESSED, info.PRESSED] = ros.internal.ros.messages.ros.default_type('int8',1, 1);
[data.UNPRESSED, info.UNPRESSED] = ros.internal.ros.messages.ros.default_type('int8',1, 0);
info.MessageType = 'baxter_core_msgs/DigitalIOState';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'state';
info.MatPath{2} = 'isInputOnly';
info.MatPath{3} = 'OFF';
info.MatPath{4} = 'ON';
info.MatPath{5} = 'PRESSED';
info.MatPath{6} = 'UNPRESSED';
