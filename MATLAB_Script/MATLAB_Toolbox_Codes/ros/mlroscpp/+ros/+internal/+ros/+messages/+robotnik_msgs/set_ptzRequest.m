function [data, info] = set_ptzRequest
%set_ptz gives an empty data for robotnik_msgs/set_ptzRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/set_ptzRequest';
[data.Pan, info.Pan] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Tilt, info.Tilt] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Zoom, info.Zoom] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Relative, info.Relative] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'robotnik_msgs/set_ptzRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'pan';
info.MatPath{2} = 'tilt';
info.MatPath{3} = 'zoom';
info.MatPath{4} = 'relative';
