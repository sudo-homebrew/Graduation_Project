function [data, info] = icon
%Icon gives an empty data for rocon_std_msgs/Icon

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rocon_std_msgs/Icon';
[data.ResourceName, info.ResourceName] = ros.internal.ros.messages.ros.char('string',0);
[data.Format, info.Format] = ros.internal.ros.messages.ros.char('string',0);
[data.Data, info.Data] = ros.internal.ros.messages.ros.default_type('uint8',NaN);
info.MessageType = 'rocon_std_msgs/Icon';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'resource_name';
info.MatPath{2} = 'format';
info.MatPath{3} = 'data';
