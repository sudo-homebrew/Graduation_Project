function [data, info] = setProfileResponse
%SetProfile gives an empty data for pr2_mechanism_controllers/SetProfileResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_mechanism_controllers/SetProfileResponse';
[data.Time, info.Time] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'pr2_mechanism_controllers/SetProfileResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'time';
