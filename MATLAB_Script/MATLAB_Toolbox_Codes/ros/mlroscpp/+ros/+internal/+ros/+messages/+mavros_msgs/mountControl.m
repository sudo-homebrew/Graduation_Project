function [data, info] = mountControl
%MountControl gives an empty data for mavros_msgs/MountControl

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/MountControl';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Mode, info.Mode] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.MAVMOUNTMODERETRACT, info.MAVMOUNTMODERETRACT] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.MAVMOUNTMODENEUTRAL, info.MAVMOUNTMODENEUTRAL] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.MAVMOUNTMODEMAVLINKTARGETING, info.MAVMOUNTMODEMAVLINKTARGETING] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.MAVMOUNTMODERCTARGETING, info.MAVMOUNTMODERCTARGETING] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.MAVMOUNTMODEGPSPOINT, info.MAVMOUNTMODEGPSPOINT] = ros.internal.ros.messages.ros.default_type('uint8',1, 4);
[data.Pitch, info.Pitch] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Roll, info.Roll] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Yaw, info.Yaw] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Altitude, info.Altitude] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Latitude, info.Latitude] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Longitude, info.Longitude] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'mavros_msgs/MountControl';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,18);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'mode';
info.MatPath{8} = 'MAV_MOUNT_MODE_RETRACT';
info.MatPath{9} = 'MAV_MOUNT_MODE_NEUTRAL';
info.MatPath{10} = 'MAV_MOUNT_MODE_MAVLINK_TARGETING';
info.MatPath{11} = 'MAV_MOUNT_MODE_RC_TARGETING';
info.MatPath{12} = 'MAV_MOUNT_MODE_GPS_POINT';
info.MatPath{13} = 'pitch';
info.MatPath{14} = 'roll';
info.MatPath{15} = 'yaw';
info.MatPath{16} = 'altitude';
info.MatPath{17} = 'latitude';
info.MatPath{18} = 'longitude';