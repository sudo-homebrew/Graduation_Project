function [data, info] = uint8Array3TestMessage
%Uint8Array3TestMessage gives an empty data for rospy_message_converter/Uint8Array3TestMessage

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rospy_message_converter/Uint8Array3TestMessage';
[data.Data, info.Data] = ros.internal.ros.messages.ros.default_type('uint8',3);
info.MessageType = 'rospy_message_converter/Uint8Array3TestMessage';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'data';
