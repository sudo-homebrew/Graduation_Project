function [data, info] = programControl
%ProgramControl gives an empty data for applanix_msgs/ProgramControl

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/ProgramControl';
[data.Transaction, info.Transaction] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.CONTROLALIVE, info.CONTROLALIVE] = ros.internal.ros.messages.ros.default_type('uint16',1, 0);
[data.CONTROLTERMINATETCPIP, info.CONTROLTERMINATETCPIP] = ros.internal.ros.messages.ros.default_type('uint16',1, 1);
[data.CONTROLRESETGAMS, info.CONTROLRESETGAMS] = ros.internal.ros.messages.ros.default_type('uint16',1, 100);
[data.CONTROLRESETPOS, info.CONTROLRESETPOS] = ros.internal.ros.messages.ros.default_type('uint16',1, 101);
[data.CONTROLSHUTDOWNPOS, info.CONTROLSHUTDOWNPOS] = ros.internal.ros.messages.ros.default_type('uint16',1, 102);
[data.Control, info.Control] = ros.internal.ros.messages.ros.default_type('uint16',1);
info.MessageType = 'applanix_msgs/ProgramControl';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'transaction';
info.MatPath{2} = 'CONTROL_ALIVE';
info.MatPath{3} = 'CONTROL_TERMINATE_TCPIP';
info.MatPath{4} = 'CONTROL_RESET_GAMS';
info.MatPath{5} = 'CONTROL_RESET_POS';
info.MatPath{6} = 'CONTROL_SHUTDOWN_POS';
info.MatPath{7} = 'control';
