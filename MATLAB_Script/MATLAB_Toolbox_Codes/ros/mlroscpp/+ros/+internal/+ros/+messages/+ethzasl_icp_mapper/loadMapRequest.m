function [data, info] = loadMapRequest
%LoadMap gives an empty data for ethzasl_icp_mapper/LoadMapRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ethzasl_icp_mapper/LoadMapRequest';
[data.Filename, info.Filename] = ros.internal.ros.messages.std_msgs.string;
info.Filename.MLdataType = 'struct';
info.MessageType = 'ethzasl_icp_mapper/LoadMapRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'filename';
info.MatPath{2} = 'filename.data';
