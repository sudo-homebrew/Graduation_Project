function [data, info] = outputGroup
%OutputGroup gives an empty data for applanix_msgs/OutputGroup

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/OutputGroup';
[data.Group, info.Group] = ros.internal.ros.messages.ros.default_type('uint16',1);
info.MessageType = 'applanix_msgs/OutputGroup';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'group';
