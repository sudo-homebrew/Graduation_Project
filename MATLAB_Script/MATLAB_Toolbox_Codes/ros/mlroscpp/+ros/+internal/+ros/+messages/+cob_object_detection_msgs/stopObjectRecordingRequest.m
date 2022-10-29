function [data, info] = stopObjectRecordingRequest
%StopObjectRecording gives an empty data for cob_object_detection_msgs/StopObjectRecordingRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_object_detection_msgs/StopObjectRecordingRequest';
[data.StopAlthoughModelIsIncomplete, info.StopAlthoughModelIsIncomplete] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'cob_object_detection_msgs/StopObjectRecordingRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'stop_although_model_is_incomplete';
