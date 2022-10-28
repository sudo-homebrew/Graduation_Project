function [data, info] = named_inputs_outputs
%named_inputs_outputs gives an empty data for robotnik_msgs/named_inputs_outputs

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/named_inputs_outputs';
[data.DigitalInputs, info.DigitalInputs] = ros.internal.ros.messages.robotnik_msgs.named_input_output;
info.DigitalInputs.MLdataType = 'struct';
info.DigitalInputs.MaxLen = NaN;
info.DigitalInputs.MinLen = 0;
data.DigitalInputs = data.DigitalInputs([],1);
[data.DigitalOutputs, info.DigitalOutputs] = ros.internal.ros.messages.robotnik_msgs.named_input_output;
info.DigitalOutputs.MLdataType = 'struct';
info.DigitalOutputs.MaxLen = NaN;
info.DigitalOutputs.MinLen = 0;
data.DigitalOutputs = data.DigitalOutputs([],1);
info.MessageType = 'robotnik_msgs/named_inputs_outputs';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'digital_inputs';
info.MatPath{2} = 'digital_inputs.name';
info.MatPath{3} = 'digital_inputs.value';
info.MatPath{4} = 'digital_outputs';
info.MatPath{5} = 'digital_outputs.name';
info.MatPath{6} = 'digital_outputs.value';
