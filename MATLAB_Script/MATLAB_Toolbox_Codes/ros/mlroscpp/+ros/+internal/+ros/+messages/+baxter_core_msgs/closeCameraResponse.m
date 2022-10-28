function [data, info] = closeCameraResponse
%CloseCamera gives an empty data for baxter_core_msgs/CloseCameraResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'baxter_core_msgs/CloseCameraResponse';
[data.Err, info.Err] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'baxter_core_msgs/CloseCameraResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'err';
