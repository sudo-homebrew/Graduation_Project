function [data, info] = changeDriftDimensionsRequest
%ChangeDriftDimensions gives an empty data for moveit_msgs/ChangeDriftDimensionsRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/ChangeDriftDimensionsRequest';
[data.DriftXTranslation, info.DriftXTranslation] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.DriftYTranslation, info.DriftYTranslation] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.DriftZTranslation, info.DriftZTranslation] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.DriftXRotation, info.DriftXRotation] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.DriftYRotation, info.DriftYRotation] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.DriftZRotation, info.DriftZRotation] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.TransformJogFrameToDriftFrame, info.TransformJogFrameToDriftFrame] = ros.internal.ros.messages.geometry_msgs.transform;
info.TransformJogFrameToDriftFrame.MLdataType = 'struct';
info.MessageType = 'moveit_msgs/ChangeDriftDimensionsRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,16);
info.MatPath{1} = 'drift_x_translation';
info.MatPath{2} = 'drift_y_translation';
info.MatPath{3} = 'drift_z_translation';
info.MatPath{4} = 'drift_x_rotation';
info.MatPath{5} = 'drift_y_rotation';
info.MatPath{6} = 'drift_z_rotation';
info.MatPath{7} = 'transform_jog_frame_to_drift_frame';
info.MatPath{8} = 'transform_jog_frame_to_drift_frame.translation';
info.MatPath{9} = 'transform_jog_frame_to_drift_frame.translation.x';
info.MatPath{10} = 'transform_jog_frame_to_drift_frame.translation.y';
info.MatPath{11} = 'transform_jog_frame_to_drift_frame.translation.z';
info.MatPath{12} = 'transform_jog_frame_to_drift_frame.rotation';
info.MatPath{13} = 'transform_jog_frame_to_drift_frame.rotation.x';
info.MatPath{14} = 'transform_jog_frame_to_drift_frame.rotation.y';
info.MatPath{15} = 'transform_jog_frame_to_drift_frame.rotation.z';
info.MatPath{16} = 'transform_jog_frame_to_drift_frame.rotation.w';
