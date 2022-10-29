function [data, info] = concertClientConfiguration
%ConcertClientConfiguration gives an empty data for concert_msgs/ConcertClientConfiguration

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'concert_msgs/ConcertClientConfiguration';
[data.Platform, info.Platform] = ros.internal.ros.messages.ros.char('string',0);
[data.System, info.System] = ros.internal.ros.messages.ros.char('string',0);
[data.Robot, info.Robot] = ros.internal.ros.messages.ros.char('string',0);
[data.App, info.App] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'concert_msgs/ConcertClientConfiguration';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'platform';
info.MatPath{2} = 'system';
info.MatPath{3} = 'robot';
info.MatPath{4} = 'app';
