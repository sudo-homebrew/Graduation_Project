function [data, info] = cameraSettings
%CameraSettings gives an empty data for baxter_core_msgs/CameraSettings

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'baxter_core_msgs/CameraSettings';
[data.Width, info.Width] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Height, info.Height] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Fps, info.Fps] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Controls, info.Controls] = ros.internal.ros.messages.baxter_core_msgs.cameraControl;
info.Controls.MLdataType = 'struct';
info.Controls.MaxLen = NaN;
info.Controls.MinLen = 0;
data.Controls = data.Controls([],1);
info.MessageType = 'baxter_core_msgs/CameraSettings';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,16);
info.MatPath{1} = 'width';
info.MatPath{2} = 'height';
info.MatPath{3} = 'fps';
info.MatPath{4} = 'controls';
info.MatPath{5} = 'controls.id';
info.MatPath{6} = 'controls.value';
info.MatPath{7} = 'controls.CAMERA_CONTROL_EXPOSURE';
info.MatPath{8} = 'controls.CAMERA_CONTROL_GAIN';
info.MatPath{9} = 'controls.CAMERA_CONTROL_WHITE_BALANCE_R';
info.MatPath{10} = 'controls.CAMERA_CONTROL_WHITE_BALANCE_G';
info.MatPath{11} = 'controls.CAMERA_CONTROL_WHITE_BALANCE_B';
info.MatPath{12} = 'controls.CAMERA_CONTROL_WINDOW_X';
info.MatPath{13} = 'controls.CAMERA_CONTROL_WINDOW_Y';
info.MatPath{14} = 'controls.CAMERA_CONTROL_FLIP';
info.MatPath{15} = 'controls.CAMERA_CONTROL_MIRROR';
info.MatPath{16} = 'controls.CAMERA_CONTROL_RESOLUTION_HALF';
