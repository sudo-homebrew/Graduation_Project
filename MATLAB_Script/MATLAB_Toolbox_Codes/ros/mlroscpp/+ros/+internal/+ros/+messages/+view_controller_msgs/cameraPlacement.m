function [data, info] = cameraPlacement
%CameraPlacement gives an empty data for view_controller_msgs/CameraPlacement

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'view_controller_msgs/CameraPlacement';
[data.InterpolationMode, info.InterpolationMode] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.LINEAR, info.LINEAR] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.SPHERICAL, info.SPHERICAL] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.TargetFrame, info.TargetFrame] = ros.internal.ros.messages.ros.char('string',0);
[data.TimeFromStart, info.TimeFromStart] = ros.internal.ros.messages.ros.duration;
info.TimeFromStart.MLdataType = 'struct';
[data.Eye, info.Eye] = ros.internal.ros.messages.geometry_msgs.pointStamped;
info.Eye.MLdataType = 'struct';
[data.Focus, info.Focus] = ros.internal.ros.messages.geometry_msgs.pointStamped;
info.Focus.MLdataType = 'struct';
[data.Up, info.Up] = ros.internal.ros.messages.geometry_msgs.vector3Stamped;
info.Up.MLdataType = 'struct';
[data.MouseInteractionMode, info.MouseInteractionMode] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.NOCHANGE, info.NOCHANGE] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.ORBIT, info.ORBIT] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.FPS, info.FPS] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.InteractionDisabled, info.InteractionDisabled] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.AllowFreeYawAxis, info.AllowFreeYawAxis] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'view_controller_msgs/CameraPlacement';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,46);
info.MatPath{1} = 'interpolation_mode';
info.MatPath{2} = 'LINEAR';
info.MatPath{3} = 'SPHERICAL';
info.MatPath{4} = 'target_frame';
info.MatPath{5} = 'time_from_start';
info.MatPath{6} = 'time_from_start.sec';
info.MatPath{7} = 'time_from_start.nsec';
info.MatPath{8} = 'eye';
info.MatPath{9} = 'eye.header';
info.MatPath{10} = 'eye.header.seq';
info.MatPath{11} = 'eye.header.stamp';
info.MatPath{12} = 'eye.header.stamp.sec';
info.MatPath{13} = 'eye.header.stamp.nsec';
info.MatPath{14} = 'eye.header.frame_id';
info.MatPath{15} = 'eye.point';
info.MatPath{16} = 'eye.point.x';
info.MatPath{17} = 'eye.point.y';
info.MatPath{18} = 'eye.point.z';
info.MatPath{19} = 'focus';
info.MatPath{20} = 'focus.header';
info.MatPath{21} = 'focus.header.seq';
info.MatPath{22} = 'focus.header.stamp';
info.MatPath{23} = 'focus.header.stamp.sec';
info.MatPath{24} = 'focus.header.stamp.nsec';
info.MatPath{25} = 'focus.header.frame_id';
info.MatPath{26} = 'focus.point';
info.MatPath{27} = 'focus.point.x';
info.MatPath{28} = 'focus.point.y';
info.MatPath{29} = 'focus.point.z';
info.MatPath{30} = 'up';
info.MatPath{31} = 'up.header';
info.MatPath{32} = 'up.header.seq';
info.MatPath{33} = 'up.header.stamp';
info.MatPath{34} = 'up.header.stamp.sec';
info.MatPath{35} = 'up.header.stamp.nsec';
info.MatPath{36} = 'up.header.frame_id';
info.MatPath{37} = 'up.vector';
info.MatPath{38} = 'up.vector.x';
info.MatPath{39} = 'up.vector.y';
info.MatPath{40} = 'up.vector.z';
info.MatPath{41} = 'mouse_interaction_mode';
info.MatPath{42} = 'NO_CHANGE';
info.MatPath{43} = 'ORBIT';
info.MatPath{44} = 'FPS';
info.MatPath{45} = 'interaction_disabled';
info.MatPath{46} = 'allow_free_yaw_axis';