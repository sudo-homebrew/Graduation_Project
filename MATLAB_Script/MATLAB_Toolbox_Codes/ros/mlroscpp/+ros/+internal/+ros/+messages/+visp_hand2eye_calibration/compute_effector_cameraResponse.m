function [data, info] = compute_effector_cameraResponse
%compute_effector_camera gives an empty data for visp_hand2eye_calibration/compute_effector_cameraResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'visp_hand2eye_calibration/compute_effector_cameraResponse';
[data.EffectorCamera, info.EffectorCamera] = ros.internal.ros.messages.geometry_msgs.transform;
info.EffectorCamera.MLdataType = 'struct';
info.MessageType = 'visp_hand2eye_calibration/compute_effector_cameraResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'effector_camera';
info.MatPath{2} = 'effector_camera.translation';
info.MatPath{3} = 'effector_camera.translation.x';
info.MatPath{4} = 'effector_camera.translation.y';
info.MatPath{5} = 'effector_camera.translation.z';
info.MatPath{6} = 'effector_camera.rotation';
info.MatPath{7} = 'effector_camera.rotation.x';
info.MatPath{8} = 'effector_camera.rotation.y';
info.MatPath{9} = 'effector_camera.rotation.z';
info.MatPath{10} = 'effector_camera.rotation.w';
