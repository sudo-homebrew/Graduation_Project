function [data, info] = boardConfigResponse
%BoardConfig gives an empty data for wge100_camera/BoardConfigResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'wge100_camera/BoardConfigResponse';
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'wge100_camera/BoardConfigResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'success';
