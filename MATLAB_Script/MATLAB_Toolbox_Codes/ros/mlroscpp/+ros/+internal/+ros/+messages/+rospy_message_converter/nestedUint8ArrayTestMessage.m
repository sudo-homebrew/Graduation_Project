function [data, info] = nestedUint8ArrayTestMessage
%NestedUint8ArrayTestMessage gives an empty data for rospy_message_converter/NestedUint8ArrayTestMessage

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rospy_message_converter/NestedUint8ArrayTestMessage';
[data.Arrays, info.Arrays] = ros.internal.ros.messages.rospy_message_converter.uint8ArrayTestMessage;
info.Arrays.MLdataType = 'struct';
info.Arrays.MaxLen = NaN;
info.Arrays.MinLen = 0;
data.Arrays = data.Arrays([],1);
info.MessageType = 'rospy_message_converter/NestedUint8ArrayTestMessage';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'arrays';
info.MatPath{2} = 'arrays.data';
