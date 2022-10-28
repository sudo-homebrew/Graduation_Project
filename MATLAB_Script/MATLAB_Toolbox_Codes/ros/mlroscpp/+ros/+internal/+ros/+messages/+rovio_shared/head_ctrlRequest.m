function [data, info] = head_ctrlRequest
%head_ctrl gives an empty data for rovio_shared/head_ctrlRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rovio_shared/head_ctrlRequest';
[data.HEADUP, info.HEADUP] = ros.internal.ros.messages.ros.default_type('int8',1, 11);
[data.HEADDOWN, info.HEADDOWN] = ros.internal.ros.messages.ros.default_type('int8',1, 12);
[data.HEADMIDDLE, info.HEADMIDDLE] = ros.internal.ros.messages.ros.default_type('int8',1, 13);
[data.HeadPos, info.HeadPos] = ros.internal.ros.messages.ros.default_type('int8',1);
info.MessageType = 'rovio_shared/head_ctrlRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'HEAD_UP';
info.MatPath{2} = 'HEAD_DOWN';
info.MatPath{3} = 'HEAD_MIDDLE';
info.MatPath{4} = 'head_pos';
