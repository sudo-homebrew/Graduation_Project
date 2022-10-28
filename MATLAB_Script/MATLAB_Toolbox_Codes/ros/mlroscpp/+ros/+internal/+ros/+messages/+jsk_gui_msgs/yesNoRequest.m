function [data, info] = yesNoRequest
%YesNo gives an empty data for jsk_gui_msgs/YesNoRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_gui_msgs/YesNoRequest';
[data.Message, info.Message] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'jsk_gui_msgs/YesNoRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'message';
