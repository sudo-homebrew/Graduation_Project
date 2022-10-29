function [data, info] = gNSSControl
%GNSSControl gives an empty data for applanix_msgs/GNSSControl

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/GNSSControl';
[data.Transaction, info.Transaction] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.CONTROLPRIMARYGNSSCONFIG, info.CONTROLPRIMARYGNSSCONFIG] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.CONTROLPRIMARYGNSSRESET, info.CONTROLPRIMARYGNSSRESET] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.CONTROLSECONDARYGNSSCONFIG, info.CONTROLSECONDARYGNSSCONFIG] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.CONTROLSECONDARYGNSSRESET, info.CONTROLSECONDARYGNSSRESET] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.Control, info.Control] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'applanix_msgs/GNSSControl';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'transaction';
info.MatPath{2} = 'CONTROL_PRIMARY_GNSS_CONFIG';
info.MatPath{3} = 'CONTROL_PRIMARY_GNSS_RESET';
info.MatPath{4} = 'CONTROL_SECONDARY_GNSS_CONFIG';
info.MatPath{5} = 'CONTROL_SECONDARY_GNSS_RESET';
info.MatPath{6} = 'control';
