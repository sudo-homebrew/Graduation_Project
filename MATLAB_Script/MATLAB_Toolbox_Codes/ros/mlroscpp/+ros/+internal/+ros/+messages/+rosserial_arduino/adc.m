function [data, info] = adc
%Adc gives an empty data for rosserial_arduino/Adc

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosserial_arduino/Adc';
[data.Adc0, info.Adc0] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.Adc1, info.Adc1] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.Adc2, info.Adc2] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.Adc3, info.Adc3] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.Adc4, info.Adc4] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.Adc5, info.Adc5] = ros.internal.ros.messages.ros.default_type('uint16',1);
info.MessageType = 'rosserial_arduino/Adc';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'adc0';
info.MatPath{2} = 'adc1';
info.MatPath{3} = 'adc2';
info.MatPath{4} = 'adc3';
info.MatPath{5} = 'adc4';
info.MatPath{6} = 'adc5';
