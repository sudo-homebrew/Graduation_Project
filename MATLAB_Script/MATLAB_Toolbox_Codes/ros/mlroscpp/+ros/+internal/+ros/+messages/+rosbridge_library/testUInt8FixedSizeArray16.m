function [data, info] = testUInt8FixedSizeArray16
%TestUInt8FixedSizeArray16 gives an empty data for rosbridge_library/TestUInt8FixedSizeArray16

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosbridge_library/TestUInt8FixedSizeArray16';
[data.Data, info.Data] = ros.internal.ros.messages.ros.default_type('uint8',16);
info.MessageType = 'rosbridge_library/TestUInt8FixedSizeArray16';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'data';
