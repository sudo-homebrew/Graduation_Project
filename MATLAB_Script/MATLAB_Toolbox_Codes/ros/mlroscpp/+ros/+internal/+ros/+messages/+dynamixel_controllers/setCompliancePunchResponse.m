function [data, info] = setCompliancePunchResponse
%SetCompliancePunch gives an empty data for dynamixel_controllers/SetCompliancePunchResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'dynamixel_controllers/SetCompliancePunchResponse';
info.MessageType = 'dynamixel_controllers/SetCompliancePunchResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
