function [data, info] = flightAnimRequest
%FlightAnim gives an empty data for ardrone_autonomy/FlightAnimRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ardrone_autonomy/FlightAnimRequest';
[data.Type, info.Type] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Duration, info.Duration] = ros.internal.ros.messages.ros.default_type('uint32',1);
info.MessageType = 'ardrone_autonomy/FlightAnimRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'type';
info.MatPath{2} = 'duration';
