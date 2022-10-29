function [data, info] = setDigitalSensorResponse
%SetDigitalSensor gives an empty data for cob_phidgets/SetDigitalSensorResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_phidgets/SetDigitalSensorResponse';
[data.Uri, info.Uri] = ros.internal.ros.messages.ros.char('string',0);
[data.State, info.State] = ros.internal.ros.messages.ros.default_type('int8',1);
info.MessageType = 'cob_phidgets/SetDigitalSensorResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'uri';
info.MatPath{2} = 'state';
