function [data, info] = setDigitalOutputsRequest
%SetDigitalOutputs gives an empty data for create_node/SetDigitalOutputsRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'create_node/SetDigitalOutputsRequest';
[data.DigitalOut0, info.DigitalOut0] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.DigitalOut1, info.DigitalOut1] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.DigitalOut2, info.DigitalOut2] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'create_node/SetDigitalOutputsRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'digital_out_0';
info.MatPath{2} = 'digital_out_1';
info.MatPath{3} = 'digital_out_2';
