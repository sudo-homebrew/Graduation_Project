function [data, info] = setLightModeResult
%SetLightModeResult gives an empty data for cob_light/SetLightModeResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_light/SetLightModeResult';
[data.ActiveMode, info.ActiveMode] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.ActivePriority, info.ActivePriority] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.TrackId, info.TrackId] = ros.internal.ros.messages.ros.default_type('uint64',1);
info.MessageType = 'cob_light/SetLightModeResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'active_mode';
info.MatPath{2} = 'active_priority';
info.MatPath{3} = 'track_id';
