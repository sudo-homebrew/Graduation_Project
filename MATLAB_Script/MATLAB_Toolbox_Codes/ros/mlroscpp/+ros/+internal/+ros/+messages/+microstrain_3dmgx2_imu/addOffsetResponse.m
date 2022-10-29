function [data, info] = addOffsetResponse
%AddOffset gives an empty data for microstrain_3dmgx2_imu/AddOffsetResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'microstrain_3dmgx2_imu/AddOffsetResponse';
[data.TotalOffset, info.TotalOffset] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'microstrain_3dmgx2_imu/AddOffsetResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'total_offset';
