function [data, info] = stopLightModeRequest
%StopLightMode gives an empty data for cob_light/StopLightModeRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_light/StopLightModeRequest';
[data.TrackId, info.TrackId] = ros.internal.ros.messages.ros.default_type('uint64',1);
info.MessageType = 'cob_light/StopLightModeRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'track_id';
