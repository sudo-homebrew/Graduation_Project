function [data, info] = multiWaveformTransition
%MultiWaveformTransition gives an empty data for ethercat_trigger_controllers/MultiWaveformTransition

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ethercat_trigger_controllers/MultiWaveformTransition';
[data.Time, info.Time] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Value, info.Value] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Topic, info.Topic] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'ethercat_trigger_controllers/MultiWaveformTransition';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'time';
info.MatPath{2} = 'value';
info.MatPath{3} = 'topic';
