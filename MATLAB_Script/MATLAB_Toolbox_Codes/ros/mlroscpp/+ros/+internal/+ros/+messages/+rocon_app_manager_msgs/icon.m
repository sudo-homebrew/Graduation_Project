function [data, info] = icon
%Icon gives an empty data for rocon_app_manager_msgs/Icon

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rocon_app_manager_msgs/Icon';
[data.Format, info.Format] = ros.internal.ros.messages.ros.char('string',0);
[data.Data, info.Data] = ros.internal.ros.messages.ros.default_type('uint8',NaN);
info.MessageType = 'rocon_app_manager_msgs/Icon';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'format';
info.MatPath{2} = 'data';