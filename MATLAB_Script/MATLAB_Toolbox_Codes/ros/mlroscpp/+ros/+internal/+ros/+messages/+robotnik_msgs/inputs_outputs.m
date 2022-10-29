function [data, info] = inputs_outputs
%inputs_outputs gives an empty data for robotnik_msgs/inputs_outputs

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/inputs_outputs';
[data.DigitalInputs, info.DigitalInputs] = ros.internal.ros.messages.ros.default_type('logical',NaN);
[data.DigitalOutputs, info.DigitalOutputs] = ros.internal.ros.messages.ros.default_type('logical',NaN);
[data.AnalogInputs, info.AnalogInputs] = ros.internal.ros.messages.ros.default_type('single',NaN);
[data.AnalogOutputs, info.AnalogOutputs] = ros.internal.ros.messages.ros.default_type('single',NaN);
info.MessageType = 'robotnik_msgs/inputs_outputs';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'digital_inputs';
info.MatPath{2} = 'digital_outputs';
info.MatPath{3} = 'analog_inputs';
info.MatPath{4} = 'analog_outputs';
