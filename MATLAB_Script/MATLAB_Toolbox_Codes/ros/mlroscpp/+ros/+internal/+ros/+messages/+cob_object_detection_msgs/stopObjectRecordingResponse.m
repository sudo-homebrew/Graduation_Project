function [data, info] = stopObjectRecordingResponse
%StopObjectRecording gives an empty data for cob_object_detection_msgs/StopObjectRecordingResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_object_detection_msgs/StopObjectRecordingResponse';
[data.RecordingStopped, info.RecordingStopped] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'cob_object_detection_msgs/StopObjectRecordingResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'recording_stopped';
