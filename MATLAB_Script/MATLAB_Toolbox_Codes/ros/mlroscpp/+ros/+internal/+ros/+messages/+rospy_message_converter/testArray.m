function [data, info] = testArray
%TestArray gives an empty data for rospy_message_converter/TestArray

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rospy_message_converter/TestArray';
[data.Data, info.Data] = ros.internal.ros.messages.ros.default_type('double',NaN);
info.MessageType = 'rospy_message_converter/TestArray';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'data';
