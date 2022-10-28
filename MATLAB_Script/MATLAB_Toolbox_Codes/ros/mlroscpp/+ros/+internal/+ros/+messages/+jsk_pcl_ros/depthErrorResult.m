function [data, info] = depthErrorResult
%DepthErrorResult gives an empty data for jsk_pcl_ros/DepthErrorResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_pcl_ros/DepthErrorResult';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.U, info.U] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.V, info.V] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.TrueDepth, info.TrueDepth] = ros.internal.ros.messages.ros.default_type('single',1);
[data.ObservedDepth, info.ObservedDepth] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'jsk_pcl_ros/DepthErrorResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'u';
info.MatPath{8} = 'v';
info.MatPath{9} = 'true_depth';
info.MatPath{10} = 'observed_depth';
