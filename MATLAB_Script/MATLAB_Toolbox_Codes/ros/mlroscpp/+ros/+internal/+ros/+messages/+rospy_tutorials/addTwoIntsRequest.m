function [data, info] = addTwoIntsRequest
%AddTwoInts gives an empty data for rospy_tutorials/AddTwoIntsRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rospy_tutorials/AddTwoIntsRequest';
[data.A, info.A] = ros.internal.ros.messages.ros.default_type('int64',1);
[data.B, info.B] = ros.internal.ros.messages.ros.default_type('int64',1);
info.MessageType = 'rospy_tutorials/AddTwoIntsRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'a';
info.MatPath{2} = 'b';
