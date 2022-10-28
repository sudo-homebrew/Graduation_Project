function [data, info] = setBuzzerRequest
%SetBuzzer gives an empty data for robotnik_msgs/SetBuzzerRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/SetBuzzerRequest';
[data.Enable, info.Enable] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.BeepFreq, info.BeepFreq] = ros.internal.ros.messages.ros.default_type('double',1);
[data.TimeEnabled, info.TimeEnabled] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'robotnik_msgs/SetBuzzerRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'enable';
info.MatPath{2} = 'beep_freq';
info.MatPath{3} = 'time_enabled';
