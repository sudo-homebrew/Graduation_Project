function [data, info] = bagTrainObjectResponse
%BagTrainObject gives an empty data for cob_object_detection_msgs/BagTrainObjectResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_object_detection_msgs/BagTrainObjectResponse';
[data.Trained, info.Trained] = ros.internal.ros.messages.std_msgs.string;
info.Trained.MLdataType = 'struct';
info.MessageType = 'cob_object_detection_msgs/BagTrainObjectResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'trained';
info.MatPath{2} = 'trained.data';
