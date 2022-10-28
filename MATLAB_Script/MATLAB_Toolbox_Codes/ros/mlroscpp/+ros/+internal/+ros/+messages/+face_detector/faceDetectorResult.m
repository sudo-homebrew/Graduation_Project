function [data, info] = faceDetectorResult
%FaceDetectorResult gives an empty data for face_detector/FaceDetectorResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'face_detector/FaceDetectorResult';
[data.FacePositions, info.FacePositions] = ros.internal.ros.messages.people_msgs.positionMeasurement;
info.FacePositions.MLdataType = 'struct';
info.FacePositions.MaxLen = NaN;
info.FacePositions.MinLen = 0;
data.FacePositions = data.FacePositions([],1);
info.MessageType = 'face_detector/FaceDetectorResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,16);
info.MatPath{1} = 'face_positions';
info.MatPath{2} = 'face_positions.header';
info.MatPath{3} = 'face_positions.header.seq';
info.MatPath{4} = 'face_positions.header.stamp';
info.MatPath{5} = 'face_positions.header.stamp.sec';
info.MatPath{6} = 'face_positions.header.stamp.nsec';
info.MatPath{7} = 'face_positions.header.frame_id';
info.MatPath{8} = 'face_positions.name';
info.MatPath{9} = 'face_positions.object_id';
info.MatPath{10} = 'face_positions.pos';
info.MatPath{11} = 'face_positions.pos.x';
info.MatPath{12} = 'face_positions.pos.y';
info.MatPath{13} = 'face_positions.pos.z';
info.MatPath{14} = 'face_positions.reliability';
info.MatPath{15} = 'face_positions.covariance';
info.MatPath{16} = 'face_positions.initialization';
