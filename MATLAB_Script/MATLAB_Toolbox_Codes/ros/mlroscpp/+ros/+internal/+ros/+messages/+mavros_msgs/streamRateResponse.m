function [data, info] = streamRateResponse
%StreamRate gives an empty data for mavros_msgs/StreamRateResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/StreamRateResponse';
info.MessageType = 'mavros_msgs/StreamRateResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
