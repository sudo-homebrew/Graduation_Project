function [data, info] = overrideRCIn
%OverrideRCIn gives an empty data for mavros_msgs/OverrideRCIn

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/OverrideRCIn';
[data.CHANRELEASE, info.CHANRELEASE] = ros.internal.ros.messages.ros.default_type('uint16',1, 0);
[data.CHANNOCHANGE, info.CHANNOCHANGE] = ros.internal.ros.messages.ros.default_type('uint16',1, 65535);
[data.Channels, info.Channels] = ros.internal.ros.messages.ros.default_type('uint16',8);
info.MessageType = 'mavros_msgs/OverrideRCIn';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'CHAN_RELEASE';
info.MatPath{2} = 'CHAN_NOCHANGE';
info.MatPath{3} = 'channels';
