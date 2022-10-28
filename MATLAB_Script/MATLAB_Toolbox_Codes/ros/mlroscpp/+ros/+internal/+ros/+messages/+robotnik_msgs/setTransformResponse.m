function [data, info] = setTransformResponse
%SetTransform gives an empty data for robotnik_msgs/SetTransformResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/SetTransformResponse';
[data.Ret, info.Ret] = ros.internal.ros.messages.robotnik_msgs.returnMessage;
info.Ret.MLdataType = 'struct';
info.MessageType = 'robotnik_msgs/SetTransformResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'ret';
info.MatPath{2} = 'ret.success';
info.MatPath{3} = 'ret.message';
info.MatPath{4} = 'ret.code';
