function [data, info] = setCompliancePunchRequest
%SetCompliancePunch gives an empty data for dynamixel_controllers/SetCompliancePunchRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'dynamixel_controllers/SetCompliancePunchRequest';
[data.Punch, info.Punch] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'dynamixel_controllers/SetCompliancePunchRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'punch';
