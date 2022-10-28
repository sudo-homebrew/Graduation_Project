function [data, info] = wav_playRequest
%wav_play gives an empty data for rovio_shared/wav_playRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rovio_shared/wav_playRequest';
[data.F, info.F] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'rovio_shared/wav_playRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'f';
