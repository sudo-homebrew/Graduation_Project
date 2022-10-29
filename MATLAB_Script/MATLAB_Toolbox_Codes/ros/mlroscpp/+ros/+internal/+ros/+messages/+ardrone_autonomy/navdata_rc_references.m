function [data, info] = navdata_rc_references
%navdata_rc_references gives an empty data for ardrone_autonomy/navdata_rc_references

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ardrone_autonomy/navdata_rc_references';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.DroneTime, info.DroneTime] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Tag, info.Tag] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.Size, info.Size] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.RcRefPitch, info.RcRefPitch] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.RcRefRoll, info.RcRefRoll] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.RcRefYaw, info.RcRefYaw] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.RcRefGaz, info.RcRefGaz] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.RcRefAg, info.RcRefAg] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'ardrone_autonomy/navdata_rc_references';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'drone_time';
info.MatPath{8} = 'tag';
info.MatPath{9} = 'size';
info.MatPath{10} = 'rc_ref_pitch';
info.MatPath{11} = 'rc_ref_roll';
info.MatPath{12} = 'rc_ref_yaw';
info.MatPath{13} = 'rc_ref_gaz';
info.MatPath{14} = 'rc_ref_ag';
