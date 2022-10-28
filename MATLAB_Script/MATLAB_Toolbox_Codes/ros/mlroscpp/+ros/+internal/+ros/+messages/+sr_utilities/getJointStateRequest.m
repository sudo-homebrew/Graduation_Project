function [data, info] = getJointStateRequest
%getJointState gives an empty data for sr_utilities/getJointStateRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_utilities/getJointStateRequest';
info.MessageType = 'sr_utilities/getJointStateRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
