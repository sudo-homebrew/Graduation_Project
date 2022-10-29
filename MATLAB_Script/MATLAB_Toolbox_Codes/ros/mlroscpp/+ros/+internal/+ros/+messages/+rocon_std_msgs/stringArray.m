function [data, info] = stringArray
%StringArray gives an empty data for rocon_std_msgs/StringArray

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rocon_std_msgs/StringArray';
[data.Strings, info.Strings] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'rocon_std_msgs/StringArray';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'strings';
