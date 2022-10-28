function [data, info] = setComplianceSlopeRequest
%SetComplianceSlope gives an empty data for dynamixel_controllers/SetComplianceSlopeRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'dynamixel_controllers/SetComplianceSlopeRequest';
[data.Slope, info.Slope] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'dynamixel_controllers/SetComplianceSlopeRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'slope';
