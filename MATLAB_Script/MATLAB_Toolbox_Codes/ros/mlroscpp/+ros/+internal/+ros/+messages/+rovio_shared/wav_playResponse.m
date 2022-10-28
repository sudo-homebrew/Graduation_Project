function [data, info] = wav_playResponse
%wav_play gives an empty data for rovio_shared/wav_playResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rovio_shared/wav_playResponse';
info.MessageType = 'rovio_shared/wav_playResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
