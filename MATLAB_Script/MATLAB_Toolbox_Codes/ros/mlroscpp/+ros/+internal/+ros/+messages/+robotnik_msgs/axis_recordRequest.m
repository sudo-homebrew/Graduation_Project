function [data, info] = axis_recordRequest
%axis_record gives an empty data for robotnik_msgs/axis_recordRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/axis_recordRequest';
[data.Action, info.Action] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.Directory, info.Directory] = ros.internal.ros.messages.ros.char('string',0);
[data.Profile, info.Profile] = ros.internal.ros.messages.ros.char('string',0);
[data.Id, info.Id] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'robotnik_msgs/axis_recordRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'action';
info.MatPath{2} = 'directory';
info.MatPath{3} = 'profile';
info.MatPath{4} = 'id';
