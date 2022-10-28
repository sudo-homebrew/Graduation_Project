function [data, info] = playTuneV2
%PlayTuneV2 gives an empty data for mavros_msgs/PlayTuneV2

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/PlayTuneV2';
[data.QBASIC11, info.QBASIC11] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.MMLMODERN, info.MMLMODERN] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.Format, info.Format] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Tune, info.Tune] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'mavros_msgs/PlayTuneV2';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'QBASIC1_1';
info.MatPath{2} = 'MML_MODERN';
info.MatPath{3} = 'format';
info.MatPath{4} = 'tune';
