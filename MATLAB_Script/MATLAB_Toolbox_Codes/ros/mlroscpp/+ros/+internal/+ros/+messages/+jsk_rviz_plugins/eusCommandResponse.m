function [data, info] = eusCommandResponse
%EusCommand gives an empty data for jsk_rviz_plugins/EusCommandResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_rviz_plugins/EusCommandResponse';
info.MessageType = 'jsk_rviz_plugins/EusCommandResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
