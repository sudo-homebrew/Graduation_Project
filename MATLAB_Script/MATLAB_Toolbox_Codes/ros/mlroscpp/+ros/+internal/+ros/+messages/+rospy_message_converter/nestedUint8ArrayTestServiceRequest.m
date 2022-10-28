function [data, info] = nestedUint8ArrayTestServiceRequest
%NestedUint8ArrayTestService gives an empty data for rospy_message_converter/NestedUint8ArrayTestServiceRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rospy_message_converter/NestedUint8ArrayTestServiceRequest';
[data.Input, info.Input] = ros.internal.ros.messages.rospy_message_converter.nestedUint8ArrayTestMessage;
info.Input.MLdataType = 'struct';
info.MessageType = 'rospy_message_converter/NestedUint8ArrayTestServiceRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'input';
info.MatPath{2} = 'input.arrays';
info.MatPath{3} = 'input.arrays.data';
