function [data, info] = saveRecordedObjectRequest
%SaveRecordedObject gives an empty data for cob_object_detection_msgs/SaveRecordedObjectRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_object_detection_msgs/SaveRecordedObjectRequest';
[data.StorageLocation, info.StorageLocation] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'cob_object_detection_msgs/SaveRecordedObjectRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'storage_location';
