function [data, info] = compressedAngleVectorPR2
%CompressedAngleVectorPR2 gives an empty data for jsk_network_tools/CompressedAngleVectorPR2

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_network_tools/CompressedAngleVectorPR2';
[data.Angles, info.Angles] = ros.internal.ros.messages.ros.default_type('uint8',17);
info.MessageType = 'jsk_network_tools/CompressedAngleVectorPR2';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'angles';
