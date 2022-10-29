function [data, info] = objectFitCommand
%ObjectFitCommand gives an empty data for jsk_rviz_plugins/ObjectFitCommand

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_rviz_plugins/ObjectFitCommand';
[data.FIT, info.FIT] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.NEAR, info.NEAR] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.OTHER, info.OTHER] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.REVERSEFIT, info.REVERSEFIT] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.REVERSENEAR, info.REVERSENEAR] = ros.internal.ros.messages.ros.default_type('uint8',1, 4);
[data.REVERSEOTHER, info.REVERSEOTHER] = ros.internal.ros.messages.ros.default_type('uint8',1, 5);
[data.Command, info.Command] = ros.internal.ros.messages.ros.default_type('int8',1);
info.MessageType = 'jsk_rviz_plugins/ObjectFitCommand';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'FIT';
info.MatPath{2} = 'NEAR';
info.MatPath{3} = 'OTHER';
info.MatPath{4} = 'REVERSE_FIT';
info.MatPath{5} = 'REVERSE_NEAR';
info.MatPath{6} = 'REVERSE_OTHER';
info.MatPath{7} = 'command';
