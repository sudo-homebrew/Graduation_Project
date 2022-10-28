function [data, info] = linkState
%LinkState gives an empty data for multimaster_msgs_fkie/LinkState

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'multimaster_msgs_fkie/LinkState';
[data.Destination, info.Destination] = ros.internal.ros.messages.ros.char('string',0);
[data.Quality, info.Quality] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'multimaster_msgs_fkie/LinkState';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'destination';
info.MatPath{2} = 'quality';
