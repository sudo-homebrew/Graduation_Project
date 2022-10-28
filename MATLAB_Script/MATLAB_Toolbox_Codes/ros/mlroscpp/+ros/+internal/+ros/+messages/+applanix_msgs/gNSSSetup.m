function [data, info] = gNSSSetup
%GNSSSetup gives an empty data for applanix_msgs/GNSSSetup

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/GNSSSetup';
[data.Transaction, info.Transaction] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.AUTOCONFIGENABLE, info.AUTOCONFIGENABLE] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.AUTOCONFIGDISABLE, info.AUTOCONFIGDISABLE] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.Autoconfig, info.Autoconfig] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Rate, info.Rate] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Reserved1, info.Reserved1] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Reserved2, info.Reserved2] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Reserved3, info.Reserved3] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'applanix_msgs/GNSSSetup';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'transaction';
info.MatPath{2} = 'AUTOCONFIG_ENABLE';
info.MatPath{3} = 'AUTOCONFIG_DISABLE';
info.MatPath{4} = 'autoconfig';
info.MatPath{5} = 'rate';
info.MatPath{6} = 'reserved1';
info.MatPath{7} = 'reserved2';
info.MatPath{8} = 'reserved3';
