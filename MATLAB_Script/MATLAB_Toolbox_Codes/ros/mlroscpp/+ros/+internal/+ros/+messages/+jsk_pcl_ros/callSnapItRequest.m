function [data, info] = callSnapItRequest
%CallSnapIt gives an empty data for jsk_pcl_ros/CallSnapItRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_pcl_ros/CallSnapItRequest';
[data.Request, info.Request] = ros.internal.ros.messages.jsk_pcl_ros.snapItRequest;
info.Request.MLdataType = 'struct';
info.MessageType = 'jsk_pcl_ros/CallSnapItRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,48);
info.MatPath{1} = 'request';
info.MatPath{2} = 'request.header';
info.MatPath{3} = 'request.header.seq';
info.MatPath{4} = 'request.header.stamp';
info.MatPath{5} = 'request.header.stamp.sec';
info.MatPath{6} = 'request.header.stamp.nsec';
info.MatPath{7} = 'request.header.frame_id';
info.MatPath{8} = 'request.MODEL_PLANE';
info.MatPath{9} = 'request.MODEL_CYLINDER';
info.MatPath{10} = 'request.model_type';
info.MatPath{11} = 'request.target_plane';
info.MatPath{12} = 'request.target_plane.header';
info.MatPath{13} = 'request.target_plane.header.seq';
info.MatPath{14} = 'request.target_plane.header.stamp';
info.MatPath{15} = 'request.target_plane.header.stamp.sec';
info.MatPath{16} = 'request.target_plane.header.stamp.nsec';
info.MatPath{17} = 'request.target_plane.header.frame_id';
info.MatPath{18} = 'request.target_plane.polygon';
info.MatPath{19} = 'request.target_plane.polygon.points';
info.MatPath{20} = 'request.target_plane.polygon.points.x';
info.MatPath{21} = 'request.target_plane.polygon.points.y';
info.MatPath{22} = 'request.target_plane.polygon.points.z';
info.MatPath{23} = 'request.center';
info.MatPath{24} = 'request.center.header';
info.MatPath{25} = 'request.center.header.seq';
info.MatPath{26} = 'request.center.header.stamp';
info.MatPath{27} = 'request.center.header.stamp.sec';
info.MatPath{28} = 'request.center.header.stamp.nsec';
info.MatPath{29} = 'request.center.header.frame_id';
info.MatPath{30} = 'request.center.point';
info.MatPath{31} = 'request.center.point.x';
info.MatPath{32} = 'request.center.point.y';
info.MatPath{33} = 'request.center.point.z';
info.MatPath{34} = 'request.direction';
info.MatPath{35} = 'request.direction.header';
info.MatPath{36} = 'request.direction.header.seq';
info.MatPath{37} = 'request.direction.header.stamp';
info.MatPath{38} = 'request.direction.header.stamp.sec';
info.MatPath{39} = 'request.direction.header.stamp.nsec';
info.MatPath{40} = 'request.direction.header.frame_id';
info.MatPath{41} = 'request.direction.vector';
info.MatPath{42} = 'request.direction.vector.x';
info.MatPath{43} = 'request.direction.vector.y';
info.MatPath{44} = 'request.direction.vector.z';
info.MatPath{45} = 'request.radius';
info.MatPath{46} = 'request.height';
info.MatPath{47} = 'request.max_distance';
info.MatPath{48} = 'request.eps_angle';
