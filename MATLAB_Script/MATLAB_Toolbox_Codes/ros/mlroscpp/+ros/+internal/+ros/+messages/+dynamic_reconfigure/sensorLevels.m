function [data, info] = sensorLevels
%SensorLevels gives an empty data for dynamic_reconfigure/SensorLevels

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'dynamic_reconfigure/SensorLevels';
[data.RECONFIGURECLOSE, info.RECONFIGURECLOSE] = ros.internal.ros.messages.ros.default_type('int8',1, 3);
[data.RECONFIGURESTOP, info.RECONFIGURESTOP] = ros.internal.ros.messages.ros.default_type('int8',1, 1);
[data.RECONFIGURERUNNING, info.RECONFIGURERUNNING] = ros.internal.ros.messages.ros.default_type('int8',1, 0);
info.MessageType = 'dynamic_reconfigure/SensorLevels';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'RECONFIGURE_CLOSE';
info.MatPath{2} = 'RECONFIGURE_STOP';
info.MatPath{3} = 'RECONFIGURE_RUNNING';
