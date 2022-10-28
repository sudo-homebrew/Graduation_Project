function [data, info] = groups
%Groups gives an empty data for applanix_msgs/Groups

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/Groups';
[data.Groups_, info.Groups_] = ros.internal.ros.messages.ros.default_type('uint16',NaN);
info.MessageType = 'applanix_msgs/Groups';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'groups';
