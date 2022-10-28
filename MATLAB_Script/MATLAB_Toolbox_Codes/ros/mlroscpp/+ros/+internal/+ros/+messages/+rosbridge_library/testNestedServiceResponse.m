function [data, info] = testNestedServiceResponse
%TestNestedService gives an empty data for rosbridge_library/TestNestedServiceResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosbridge_library/TestNestedServiceResponse';
[data.Data, info.Data] = ros.internal.ros.messages.std_msgs.float64;
info.Data.MLdataType = 'struct';
info.MessageType = 'rosbridge_library/TestNestedServiceResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'data';
info.MatPath{2} = 'data.data';
