function [data, info] = plot
%Plot gives an empty data for pr2_self_test_msgs/Plot

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_self_test_msgs/Plot';
[data.Title, info.Title] = ros.internal.ros.messages.ros.char('string',0);
[data.Image, info.Image] = ros.internal.ros.messages.ros.default_type('int8',NaN);
[data.ImageFormat, info.ImageFormat] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'pr2_self_test_msgs/Plot';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'title';
info.MatPath{2} = 'image';
info.MatPath{3} = 'image_format';
