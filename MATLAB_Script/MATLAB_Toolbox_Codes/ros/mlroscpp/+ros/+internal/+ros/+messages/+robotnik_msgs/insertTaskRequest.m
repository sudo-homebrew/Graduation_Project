function [data, info] = insertTaskRequest
%InsertTask gives an empty data for robotnik_msgs/InsertTaskRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/InsertTaskRequest';
[data.IdSubmission, info.IdSubmission] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.DescriptionTask, info.DescriptionTask] = ros.internal.ros.messages.ros.char('string',0);
[data.DatatimeStart, info.DatatimeStart] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'robotnik_msgs/InsertTaskRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'id_submission';
info.MatPath{2} = 'description_task';
info.MatPath{3} = 'datatime_start';
