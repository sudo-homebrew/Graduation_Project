function [data, info] = objectInImage
%ObjectInImage gives an empty data for image_cb_detector/ObjectInImage

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'image_cb_detector/ObjectInImage';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.ModelPoints, info.ModelPoints] = ros.internal.ros.messages.geometry_msgs.point;
info.ModelPoints.MLdataType = 'struct';
info.ModelPoints.MaxLen = NaN;
info.ModelPoints.MinLen = 0;
data.ModelPoints = data.ModelPoints([],1);
[data.ImagePoints, info.ImagePoints] = ros.internal.ros.messages.image_cb_detector.imagePoint;
info.ImagePoints.MLdataType = 'struct';
info.ImagePoints.MaxLen = NaN;
info.ImagePoints.MinLen = 0;
data.ImagePoints = data.ImagePoints([],1);
info.MessageType = 'image_cb_detector/ObjectInImage';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,13);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'model_points';
info.MatPath{8} = 'model_points.x';
info.MatPath{9} = 'model_points.y';
info.MatPath{10} = 'model_points.z';
info.MatPath{11} = 'image_points';
info.MatPath{12} = 'image_points.x';
info.MatPath{13} = 'image_points.y';
