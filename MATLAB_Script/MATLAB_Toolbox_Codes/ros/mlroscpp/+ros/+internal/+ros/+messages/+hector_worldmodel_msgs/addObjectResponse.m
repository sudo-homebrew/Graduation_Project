function [data, info] = addObjectResponse
%AddObject gives an empty data for hector_worldmodel_msgs/AddObjectResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'hector_worldmodel_msgs/AddObjectResponse';
[data.Object, info.Object] = ros.internal.ros.messages.hector_worldmodel_msgs.object;
info.Object.MLdataType = 'struct';
info.MessageType = 'hector_worldmodel_msgs/AddObjectResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,33);
info.MatPath{1} = 'object';
info.MatPath{2} = 'object.header';
info.MatPath{3} = 'object.header.seq';
info.MatPath{4} = 'object.header.stamp';
info.MatPath{5} = 'object.header.stamp.sec';
info.MatPath{6} = 'object.header.stamp.nsec';
info.MatPath{7} = 'object.header.frame_id';
info.MatPath{8} = 'object.pose';
info.MatPath{9} = 'object.pose.pose';
info.MatPath{10} = 'object.pose.pose.position';
info.MatPath{11} = 'object.pose.pose.position.x';
info.MatPath{12} = 'object.pose.pose.position.y';
info.MatPath{13} = 'object.pose.pose.position.z';
info.MatPath{14} = 'object.pose.pose.orientation';
info.MatPath{15} = 'object.pose.pose.orientation.x';
info.MatPath{16} = 'object.pose.pose.orientation.y';
info.MatPath{17} = 'object.pose.pose.orientation.z';
info.MatPath{18} = 'object.pose.pose.orientation.w';
info.MatPath{19} = 'object.pose.covariance';
info.MatPath{20} = 'object.info';
info.MatPath{21} = 'object.info.class_id';
info.MatPath{22} = 'object.info.object_id';
info.MatPath{23} = 'object.info.name';
info.MatPath{24} = 'object.info.support';
info.MatPath{25} = 'object.state';
info.MatPath{26} = 'object.state.UNKNOWN';
info.MatPath{27} = 'object.state.PENDING';
info.MatPath{28} = 'object.state.ACTIVE';
info.MatPath{29} = 'object.state.INACTIVE';
info.MatPath{30} = 'object.state.CONFIRMED';
info.MatPath{31} = 'object.state.DISCARDED';
info.MatPath{32} = 'object.state.APPROACHING';
info.MatPath{33} = 'object.state.state';