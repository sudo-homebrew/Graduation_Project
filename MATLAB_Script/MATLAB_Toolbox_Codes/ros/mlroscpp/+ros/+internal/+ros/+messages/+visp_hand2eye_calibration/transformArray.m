function [data, info] = transformArray
%TransformArray gives an empty data for visp_hand2eye_calibration/TransformArray

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'visp_hand2eye_calibration/TransformArray';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Transforms, info.Transforms] = ros.internal.ros.messages.geometry_msgs.transform;
info.Transforms.MLdataType = 'struct';
info.Transforms.MaxLen = NaN;
info.Transforms.MinLen = 0;
data.Transforms = data.Transforms([],1);
info.MessageType = 'visp_hand2eye_calibration/TransformArray';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,16);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'transforms';
info.MatPath{8} = 'transforms.translation';
info.MatPath{9} = 'transforms.translation.x';
info.MatPath{10} = 'transforms.translation.y';
info.MatPath{11} = 'transforms.translation.z';
info.MatPath{12} = 'transforms.rotation';
info.MatPath{13} = 'transforms.rotation.x';
info.MatPath{14} = 'transforms.rotation.y';
info.MatPath{15} = 'transforms.rotation.z';
info.MatPath{16} = 'transforms.rotation.w';
