function [data, info] = listControllersResponse
%ListControllers gives an empty data for pr2_mechanism_msgs/ListControllersResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_mechanism_msgs/ListControllersResponse';
[data.Controllers, info.Controllers] = ros.internal.ros.messages.ros.char('string',NaN);
[data.State, info.State] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'pr2_mechanism_msgs/ListControllersResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'controllers';
info.MatPath{2} = 'state';
