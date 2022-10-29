function [data, info] = gAMSCalibrationControl
%GAMSCalibrationControl gives an empty data for applanix_msgs/GAMSCalibrationControl

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/GAMSCalibrationControl';
[data.Transaction, info.Transaction] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.CONTROLSTOP, info.CONTROLSTOP] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.CONTROLBEGIN, info.CONTROLBEGIN] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.CONTROLSUSPEND, info.CONTROLSUSPEND] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.CONTROLFORCE, info.CONTROLFORCE] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.Control, info.Control] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'applanix_msgs/GAMSCalibrationControl';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'transaction';
info.MatPath{2} = 'CONTROL_STOP';
info.MatPath{3} = 'CONTROL_BEGIN';
info.MatPath{4} = 'CONTROL_SUSPEND';
info.MatPath{5} = 'CONTROL_FORCE';
info.MatPath{6} = 'control';
