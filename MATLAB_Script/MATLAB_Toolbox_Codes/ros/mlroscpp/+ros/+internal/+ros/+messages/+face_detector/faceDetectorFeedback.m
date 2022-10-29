function [data, info] = faceDetectorFeedback
%FaceDetectorFeedback gives an empty data for face_detector/FaceDetectorFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'face_detector/FaceDetectorFeedback';
info.MessageType = 'face_detector/FaceDetectorFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
