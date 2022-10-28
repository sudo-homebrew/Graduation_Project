function [data, info] = selfTestRequest
%SelfTest gives an empty data for diagnostic_msgs/SelfTestRequest

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'diagnostic_msgs/SelfTestRequest';
info.MessageType = 'diagnostic_msgs/SelfTestRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
