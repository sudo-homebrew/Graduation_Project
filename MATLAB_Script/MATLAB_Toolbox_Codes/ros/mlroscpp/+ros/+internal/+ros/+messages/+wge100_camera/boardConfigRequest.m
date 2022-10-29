function [data, info] = boardConfigRequest
%BoardConfig gives an empty data for wge100_camera/BoardConfigRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'wge100_camera/BoardConfigRequest';
[data.Serial, info.Serial] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Mac, info.Mac] = ros.internal.ros.messages.ros.default_type('uint8',NaN);
info.MessageType = 'wge100_camera/BoardConfigRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'serial';
info.MatPath{2} = 'mac';
