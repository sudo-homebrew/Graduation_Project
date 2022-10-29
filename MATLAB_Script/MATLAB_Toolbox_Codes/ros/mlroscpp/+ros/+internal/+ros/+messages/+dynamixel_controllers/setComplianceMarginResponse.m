function [data, info] = setComplianceMarginResponse
%SetComplianceMargin gives an empty data for dynamixel_controllers/SetComplianceMarginResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'dynamixel_controllers/SetComplianceMarginResponse';
info.MessageType = 'dynamixel_controllers/SetComplianceMarginResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
