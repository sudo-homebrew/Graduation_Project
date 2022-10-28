function [data, info] = getMotorsHeadingOffsetRequest
%GetMotorsHeadingOffset gives an empty data for robotnik_msgs/GetMotorsHeadingOffsetRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/GetMotorsHeadingOffsetRequest';
[data.Request, info.Request] = ros.internal.ros.messages.std_msgs.empty;
info.Request.MLdataType = 'struct';
info.MessageType = 'robotnik_msgs/GetMotorsHeadingOffsetRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'request';
