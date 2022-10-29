function [data, info] = remapping
%Remapping gives an empty data for rocon_std_msgs/Remapping

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rocon_std_msgs/Remapping';
[data.RemapFrom, info.RemapFrom] = ros.internal.ros.messages.ros.char('string',0);
[data.RemapTo, info.RemapTo] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'rocon_std_msgs/Remapping';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'remap_from';
info.MatPath{2} = 'remap_to';
