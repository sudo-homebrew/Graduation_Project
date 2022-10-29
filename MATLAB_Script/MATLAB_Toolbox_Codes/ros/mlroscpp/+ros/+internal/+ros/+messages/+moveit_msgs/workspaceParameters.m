function [data, info] = workspaceParameters
%WorkspaceParameters gives an empty data for moveit_msgs/WorkspaceParameters

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/WorkspaceParameters';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.MinCorner, info.MinCorner] = ros.internal.ros.messages.geometry_msgs.vector3;
info.MinCorner.MLdataType = 'struct';
[data.MaxCorner, info.MaxCorner] = ros.internal.ros.messages.geometry_msgs.vector3;
info.MaxCorner.MLdataType = 'struct';
info.MessageType = 'moveit_msgs/WorkspaceParameters';
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
info.MatPath{7} = 'min_corner';
info.MatPath{8} = 'min_corner.x';
info.MatPath{9} = 'min_corner.y';
info.MatPath{10} = 'min_corner.z';
info.MatPath{11} = 'max_corner';
info.MatPath{12} = 'max_corner.x';
info.MatPath{13} = 'max_corner.y';
info.MatPath{14} = 'max_corner.z';
