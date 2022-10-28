function [data, info] = getObjectInformationRequest
%GetObjectInformation gives an empty data for object_recognition_msgs/GetObjectInformationRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'object_recognition_msgs/GetObjectInformationRequest';
[data.Type, info.Type] = ros.internal.ros.messages.object_recognition_msgs.objectType;
info.Type.MLdataType = 'struct';
info.MessageType = 'object_recognition_msgs/GetObjectInformationRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'type';
info.MatPath{2} = 'type.key';
info.MatPath{3} = 'type.db';
