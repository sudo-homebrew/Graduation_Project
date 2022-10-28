function [data, info] = badTwoIntsResponse
%BadTwoInts gives an empty data for rospy_tutorials/BadTwoIntsResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rospy_tutorials/BadTwoIntsResponse';
[data.Sum, info.Sum] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'rospy_tutorials/BadTwoIntsResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'sum';
