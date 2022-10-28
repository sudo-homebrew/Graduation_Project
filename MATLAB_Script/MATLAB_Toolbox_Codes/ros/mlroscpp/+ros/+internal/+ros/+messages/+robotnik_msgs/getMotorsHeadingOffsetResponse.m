function [data, info] = getMotorsHeadingOffsetResponse
%GetMotorsHeadingOffset gives an empty data for robotnik_msgs/GetMotorsHeadingOffsetResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/GetMotorsHeadingOffsetResponse';
[data.Offsets, info.Offsets] = ros.internal.ros.messages.robotnik_msgs.motorHeadingOffset;
info.Offsets.MLdataType = 'struct';
info.Offsets.MaxLen = NaN;
info.Offsets.MinLen = 0;
data.Offsets = data.Offsets([],1);
info.MessageType = 'robotnik_msgs/GetMotorsHeadingOffsetResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'offsets';
info.MatPath{2} = 'offsets.motor';
info.MatPath{3} = 'offsets.value';
