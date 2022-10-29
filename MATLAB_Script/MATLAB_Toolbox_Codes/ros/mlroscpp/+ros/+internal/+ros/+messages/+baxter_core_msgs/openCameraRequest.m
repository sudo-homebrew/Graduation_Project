function [data, info] = openCameraRequest
%OpenCamera gives an empty data for baxter_core_msgs/OpenCameraRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'baxter_core_msgs/OpenCameraRequest';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Settings, info.Settings] = ros.internal.ros.messages.baxter_core_msgs.cameraSettings;
info.Settings.MLdataType = 'struct';
info.MessageType = 'baxter_core_msgs/OpenCameraRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,18);
info.MatPath{1} = 'name';
info.MatPath{2} = 'settings';
info.MatPath{3} = 'settings.width';
info.MatPath{4} = 'settings.height';
info.MatPath{5} = 'settings.fps';
info.MatPath{6} = 'settings.controls';
info.MatPath{7} = 'settings.controls.id';
info.MatPath{8} = 'settings.controls.value';
info.MatPath{9} = 'settings.controls.CAMERA_CONTROL_EXPOSURE';
info.MatPath{10} = 'settings.controls.CAMERA_CONTROL_GAIN';
info.MatPath{11} = 'settings.controls.CAMERA_CONTROL_WHITE_BALANCE_R';
info.MatPath{12} = 'settings.controls.CAMERA_CONTROL_WHITE_BALANCE_G';
info.MatPath{13} = 'settings.controls.CAMERA_CONTROL_WHITE_BALANCE_B';
info.MatPath{14} = 'settings.controls.CAMERA_CONTROL_WINDOW_X';
info.MatPath{15} = 'settings.controls.CAMERA_CONTROL_WINDOW_Y';
info.MatPath{16} = 'settings.controls.CAMERA_CONTROL_FLIP';
info.MatPath{17} = 'settings.controls.CAMERA_CONTROL_MIRROR';
info.MatPath{18} = 'settings.controls.CAMERA_CONTROL_RESOLUTION_HALF';
