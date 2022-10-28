function [data, info] = navModeControl
%NavModeControl gives an empty data for applanix_msgs/NavModeControl

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/NavModeControl';
[data.Transaction, info.Transaction] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.MODENONE, info.MODENONE] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.MODESTANDBY, info.MODESTANDBY] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.MODENAVIGATE, info.MODENAVIGATE] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.Mode, info.Mode] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'applanix_msgs/NavModeControl';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'transaction';
info.MatPath{2} = 'MODE_NONE';
info.MatPath{3} = 'MODE_STANDBY';
info.MatPath{4} = 'MODE_NAVIGATE';
info.MatPath{5} = 'mode';
