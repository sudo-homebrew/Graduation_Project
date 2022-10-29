function [data, info] = deleteSoundSourceRequest
%DeleteSoundSource gives an empty data for stdr_msgs/DeleteSoundSourceRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'stdr_msgs/DeleteSoundSourceRequest';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'stdr_msgs/DeleteSoundSourceRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'name';
