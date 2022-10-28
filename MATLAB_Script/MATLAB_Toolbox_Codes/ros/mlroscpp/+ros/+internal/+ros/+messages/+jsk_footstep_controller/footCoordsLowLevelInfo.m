function [data, info] = footCoordsLowLevelInfo
%FootCoordsLowLevelInfo gives an empty data for jsk_footstep_controller/FootCoordsLowLevelInfo

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_footstep_controller/FootCoordsLowLevelInfo';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.RawLlegForce, info.RawLlegForce] = ros.internal.ros.messages.ros.default_type('double',1);
[data.RawRlegForce, info.RawRlegForce] = ros.internal.ros.messages.ros.default_type('double',1);
[data.FilteredLlegForce, info.FilteredLlegForce] = ros.internal.ros.messages.ros.default_type('double',1);
[data.FilteredRlegForce, info.FilteredRlegForce] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Threshold, info.Threshold] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'jsk_footstep_controller/FootCoordsLowLevelInfo';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'raw_lleg_force';
info.MatPath{8} = 'raw_rleg_force';
info.MatPath{9} = 'filtered_lleg_force';
info.MatPath{10} = 'filtered_rleg_force';
info.MatPath{11} = 'threshold';
