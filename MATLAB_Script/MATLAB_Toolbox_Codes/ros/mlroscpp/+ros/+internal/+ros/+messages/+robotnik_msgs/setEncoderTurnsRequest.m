function [data, info] = setEncoderTurnsRequest
%SetEncoderTurns gives an empty data for robotnik_msgs/SetEncoderTurnsRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/SetEncoderTurnsRequest';
[data.EncoderTurns, info.EncoderTurns] = ros.internal.ros.messages.robotnik_msgs.motorHeadingOffset;
info.EncoderTurns.MLdataType = 'struct';
info.MessageType = 'robotnik_msgs/SetEncoderTurnsRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'encoder_turns';
info.MatPath{2} = 'encoder_turns.motor';
info.MatPath{3} = 'encoder_turns.value';
