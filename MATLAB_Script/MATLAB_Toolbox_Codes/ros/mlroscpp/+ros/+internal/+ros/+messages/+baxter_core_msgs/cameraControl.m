function [data, info] = cameraControl
%CameraControl gives an empty data for baxter_core_msgs/CameraControl

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'baxter_core_msgs/CameraControl';
[data.Id, info.Id] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Value, info.Value] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.CAMERACONTROLEXPOSURE, info.CAMERACONTROLEXPOSURE] = ros.internal.ros.messages.ros.default_type('int32',1, 100);
[data.CAMERACONTROLGAIN, info.CAMERACONTROLGAIN] = ros.internal.ros.messages.ros.default_type('int32',1, 101);
[data.CAMERACONTROLWHITEBALANCER, info.CAMERACONTROLWHITEBALANCER] = ros.internal.ros.messages.ros.default_type('int32',1, 102);
[data.CAMERACONTROLWHITEBALANCEG, info.CAMERACONTROLWHITEBALANCEG] = ros.internal.ros.messages.ros.default_type('int32',1, 103);
[data.CAMERACONTROLWHITEBALANCEB, info.CAMERACONTROLWHITEBALANCEB] = ros.internal.ros.messages.ros.default_type('int32',1, 104);
[data.CAMERACONTROLWINDOWX, info.CAMERACONTROLWINDOWX] = ros.internal.ros.messages.ros.default_type('int32',1, 105);
[data.CAMERACONTROLWINDOWY, info.CAMERACONTROLWINDOWY] = ros.internal.ros.messages.ros.default_type('int32',1, 106);
[data.CAMERACONTROLFLIP, info.CAMERACONTROLFLIP] = ros.internal.ros.messages.ros.default_type('int32',1, 107);
[data.CAMERACONTROLMIRROR, info.CAMERACONTROLMIRROR] = ros.internal.ros.messages.ros.default_type('int32',1, 108);
[data.CAMERACONTROLRESOLUTIONHALF, info.CAMERACONTROLRESOLUTIONHALF] = ros.internal.ros.messages.ros.default_type('int32',1, 109);
info.MessageType = 'baxter_core_msgs/CameraControl';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'id';
info.MatPath{2} = 'value';
info.MatPath{3} = 'CAMERA_CONTROL_EXPOSURE';
info.MatPath{4} = 'CAMERA_CONTROL_GAIN';
info.MatPath{5} = 'CAMERA_CONTROL_WHITE_BALANCE_R';
info.MatPath{6} = 'CAMERA_CONTROL_WHITE_BALANCE_G';
info.MatPath{7} = 'CAMERA_CONTROL_WHITE_BALANCE_B';
info.MatPath{8} = 'CAMERA_CONTROL_WINDOW_X';
info.MatPath{9} = 'CAMERA_CONTROL_WINDOW_Y';
info.MatPath{10} = 'CAMERA_CONTROL_FLIP';
info.MatPath{11} = 'CAMERA_CONTROL_MIRROR';
info.MatPath{12} = 'CAMERA_CONTROL_RESOLUTION_HALF';
