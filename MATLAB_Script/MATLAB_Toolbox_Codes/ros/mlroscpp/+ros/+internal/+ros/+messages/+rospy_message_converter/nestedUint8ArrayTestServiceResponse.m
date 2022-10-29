function [data, info] = nestedUint8ArrayTestServiceResponse
%NestedUint8ArrayTestService gives an empty data for rospy_message_converter/NestedUint8ArrayTestServiceResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rospy_message_converter/NestedUint8ArrayTestServiceResponse';
[data.Output, info.Output] = ros.internal.ros.messages.rospy_message_converter.nestedUint8ArrayTestMessage;
info.Output.MLdataType = 'struct';
info.MessageType = 'rospy_message_converter/NestedUint8ArrayTestServiceResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'output';
info.MatPath{2} = 'output.arrays';
info.MatPath{3} = 'output.arrays.data';
