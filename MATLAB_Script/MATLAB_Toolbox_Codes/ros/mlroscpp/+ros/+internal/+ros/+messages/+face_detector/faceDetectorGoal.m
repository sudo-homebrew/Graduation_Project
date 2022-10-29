function [data, info] = faceDetectorGoal
%FaceDetectorGoal gives an empty data for face_detector/FaceDetectorGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'face_detector/FaceDetectorGoal';
info.MessageType = 'face_detector/FaceDetectorGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
