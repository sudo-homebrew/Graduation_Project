function [data, info] = waypointList
%WaypointList gives an empty data for mavros_msgs/WaypointList

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/WaypointList';
[data.CurrentSeq, info.CurrentSeq] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.Waypoints, info.Waypoints] = ros.internal.ros.messages.mavros_msgs.waypoint;
info.Waypoints.MLdataType = 'struct';
info.Waypoints.MaxLen = NaN;
info.Waypoints.MinLen = 0;
data.Waypoints = data.Waypoints([],1);
info.MessageType = 'mavros_msgs/WaypointList';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,18);
info.MatPath{1} = 'current_seq';
info.MatPath{2} = 'waypoints';
info.MatPath{3} = 'waypoints.frame';
info.MatPath{4} = 'waypoints.FRAME_GLOBAL';
info.MatPath{5} = 'waypoints.FRAME_LOCAL_NED';
info.MatPath{6} = 'waypoints.FRAME_MISSION';
info.MatPath{7} = 'waypoints.FRAME_GLOBAL_REL_ALT';
info.MatPath{8} = 'waypoints.FRAME_LOCAL_ENU';
info.MatPath{9} = 'waypoints.command';
info.MatPath{10} = 'waypoints.is_current';
info.MatPath{11} = 'waypoints.autocontinue';
info.MatPath{12} = 'waypoints.param1';
info.MatPath{13} = 'waypoints.param2';
info.MatPath{14} = 'waypoints.param3';
info.MatPath{15} = 'waypoints.param4';
info.MatPath{16} = 'waypoints.x_lat';
info.MatPath{17} = 'waypoints.y_long';
info.MatPath{18} = 'waypoints.z_alt';
