function [data, info] = iCPAlignResponse
%ICPAlign gives an empty data for jsk_pcl_ros/ICPAlignResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_pcl_ros/ICPAlignResponse';
[data.Transform, info.Transform] = ros.internal.ros.messages.geometry_msgs.transformStamped;
info.Transform.MLdataType = 'struct';
info.MessageType = 'jsk_pcl_ros/ICPAlignResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,18);
info.MatPath{1} = 'transform';
info.MatPath{2} = 'transform.header';
info.MatPath{3} = 'transform.header.seq';
info.MatPath{4} = 'transform.header.stamp';
info.MatPath{5} = 'transform.header.stamp.sec';
info.MatPath{6} = 'transform.header.stamp.nsec';
info.MatPath{7} = 'transform.header.frame_id';
info.MatPath{8} = 'transform.child_frame_id';
info.MatPath{9} = 'transform.transform';
info.MatPath{10} = 'transform.transform.translation';
info.MatPath{11} = 'transform.transform.translation.x';
info.MatPath{12} = 'transform.transform.translation.y';
info.MatPath{13} = 'transform.transform.translation.z';
info.MatPath{14} = 'transform.transform.rotation';
info.MatPath{15} = 'transform.transform.rotation.x';
info.MatPath{16} = 'transform.transform.rotation.y';
info.MatPath{17} = 'transform.transform.rotation.z';
info.MatPath{18} = 'transform.transform.rotation.w';
