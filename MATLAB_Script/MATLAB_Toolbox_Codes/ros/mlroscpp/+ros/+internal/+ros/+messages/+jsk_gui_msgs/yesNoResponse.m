function [data, info] = yesNoResponse
%YesNo gives an empty data for jsk_gui_msgs/YesNoResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_gui_msgs/YesNoResponse';
[data.Yes, info.Yes] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'jsk_gui_msgs/YesNoResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'yes';
