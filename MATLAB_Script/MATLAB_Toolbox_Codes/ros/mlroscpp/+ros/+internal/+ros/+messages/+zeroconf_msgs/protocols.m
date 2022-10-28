function [data, info] = protocols
%Protocols gives an empty data for zeroconf_msgs/Protocols

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'zeroconf_msgs/Protocols';
[data.UNSPECIFIED, info.UNSPECIFIED] = ros.internal.ros.messages.ros.default_type('int8',1, 0);
[data.IPV4, info.IPV4] = ros.internal.ros.messages.ros.default_type('int8',1, 1);
[data.IPV6, info.IPV6] = ros.internal.ros.messages.ros.default_type('int8',1, 2);
info.MessageType = 'zeroconf_msgs/Protocols';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'UNSPECIFIED';
info.MatPath{2} = 'IPV4';
info.MatPath{3} = 'IPV6';
