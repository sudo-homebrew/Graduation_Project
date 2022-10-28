function [data, info] = valueList
%ValueList gives an empty data for data_vis_msgs/ValueList

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'data_vis_msgs/ValueList';
[data.Label, info.Label] = ros.internal.ros.messages.ros.char('string',0);
[data.Value1, info.Value1] = ros.internal.ros.messages.ros.char('string',NaN);
[data.Value2, info.Value2] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'data_vis_msgs/ValueList';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'label';
info.MatPath{2} = 'value1';
info.MatPath{3} = 'value2';
