function [data, info] = listControllerTypesResponse
%ListControllerTypes gives an empty data for pr2_mechanism_msgs/ListControllerTypesResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_mechanism_msgs/ListControllerTypesResponse';
[data.Types, info.Types] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'pr2_mechanism_msgs/ListControllerTypesResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'types';
