function [data, info] = companionProcessStatus
%CompanionProcessStatus gives an empty data for mavros_msgs/CompanionProcessStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/CompanionProcessStatus';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.State, info.State] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Component, info.Component] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.MAVSTATEUNINIT, info.MAVSTATEUNINIT] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.MAVSTATEBOOT, info.MAVSTATEBOOT] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.MAVSTATECALIBRATING, info.MAVSTATECALIBRATING] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.MAVSTATESTANDBY, info.MAVSTATESTANDBY] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.MAVSTATEACTIVE, info.MAVSTATEACTIVE] = ros.internal.ros.messages.ros.default_type('uint8',1, 4);
[data.MAVSTATECRITICAL, info.MAVSTATECRITICAL] = ros.internal.ros.messages.ros.default_type('uint8',1, 5);
[data.MAVSTATEEMERGENCY, info.MAVSTATEEMERGENCY] = ros.internal.ros.messages.ros.default_type('uint8',1, 6);
[data.MAVSTATEPOWEROFF, info.MAVSTATEPOWEROFF] = ros.internal.ros.messages.ros.default_type('uint8',1, 7);
[data.MAVSTATEFLIGHTTERMINATION, info.MAVSTATEFLIGHTTERMINATION] = ros.internal.ros.messages.ros.default_type('uint8',1, 8);
[data.MAVCOMPIDOBSTACLEAVOIDANCE, info.MAVCOMPIDOBSTACLEAVOIDANCE] = ros.internal.ros.messages.ros.default_type('uint8',1, 196);
[data.MAVCOMPIDVISUALINERTIALODOMETRY, info.MAVCOMPIDVISUALINERTIALODOMETRY] = ros.internal.ros.messages.ros.default_type('uint8',1, 197);
info.MessageType = 'mavros_msgs/CompanionProcessStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,19);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'state';
info.MatPath{8} = 'component';
info.MatPath{9} = 'MAV_STATE_UNINIT';
info.MatPath{10} = 'MAV_STATE_BOOT';
info.MatPath{11} = 'MAV_STATE_CALIBRATING';
info.MatPath{12} = 'MAV_STATE_STANDBY';
info.MatPath{13} = 'MAV_STATE_ACTIVE';
info.MatPath{14} = 'MAV_STATE_CRITICAL';
info.MatPath{15} = 'MAV_STATE_EMERGENCY';
info.MatPath{16} = 'MAV_STATE_POWEROFF';
info.MatPath{17} = 'MAV_STATE_FLIGHT_TERMINATION';
info.MatPath{18} = 'MAV_COMP_ID_OBSTACLE_AVOIDANCE';
info.MatPath{19} = 'MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY';