function [data, info] = voiceMessage
%VoiceMessage gives an empty data for jsk_gui_msgs/VoiceMessage

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_gui_msgs/VoiceMessage';
[data.Texts, info.Texts] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'jsk_gui_msgs/VoiceMessage';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'texts';
