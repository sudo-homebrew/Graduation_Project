function [data, info] = loadMapResponse
%LoadMap gives an empty data for ethzasl_icp_mapper/LoadMapResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ethzasl_icp_mapper/LoadMapResponse';
info.MessageType = 'ethzasl_icp_mapper/LoadMapResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
