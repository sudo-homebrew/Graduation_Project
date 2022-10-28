function [data, info] = addOffsetRequest
%AddOffset gives an empty data for microstrain_3dmgx2_imu/AddOffsetRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'microstrain_3dmgx2_imu/AddOffsetRequest';
[data.AddOffset, info.AddOffset] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'microstrain_3dmgx2_imu/AddOffsetRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'add_offset';
