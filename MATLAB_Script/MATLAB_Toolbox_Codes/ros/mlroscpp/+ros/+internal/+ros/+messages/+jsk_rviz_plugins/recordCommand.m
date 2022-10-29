function [data, info] = recordCommand
%RecordCommand gives an empty data for jsk_rviz_plugins/RecordCommand

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_rviz_plugins/RecordCommand';
[data.RECORD, info.RECORD] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.RECORDSTOP, info.RECORDSTOP] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.PLAY, info.PLAY] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.Command, info.Command] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.Target, info.Target] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'jsk_rviz_plugins/RecordCommand';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'RECORD';
info.MatPath{2} = 'RECORD_STOP';
info.MatPath{3} = 'PLAY';
info.MatPath{4} = 'command';
info.MatPath{5} = 'target';
