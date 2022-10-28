function [data, info] = getModelDescriptionResponse
%GetModelDescription gives an empty data for household_objects_database_msgs/GetModelDescriptionResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'household_objects_database_msgs/GetModelDescriptionResponse';
[data.ReturnCode, info.ReturnCode] = ros.internal.ros.messages.household_objects_database_msgs.databaseReturnCode;
info.ReturnCode.MLdataType = 'struct';
[data.Tags, info.Tags] = ros.internal.ros.messages.ros.char('string',NaN);
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Maker, info.Maker] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'household_objects_database_msgs/GetModelDescriptionResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'return_code';
info.MatPath{2} = 'return_code.UNKNOWN_ERROR';
info.MatPath{3} = 'return_code.DATABASE_NOT_CONNECTED';
info.MatPath{4} = 'return_code.DATABASE_QUERY_ERROR';
info.MatPath{5} = 'return_code.SUCCESS';
info.MatPath{6} = 'return_code.code';
info.MatPath{7} = 'tags';
info.MatPath{8} = 'name';
info.MatPath{9} = 'maker';
