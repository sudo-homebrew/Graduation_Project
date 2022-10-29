function [data, info] = set_modeRequest
%set_mode gives an empty data for robotnik_msgs/set_modeRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/set_modeRequest';
[data.Mode, info.Mode] = ros.internal.ros.messages.ros.default_type('int8',1);
info.MessageType = 'robotnik_msgs/set_modeRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'mode';
