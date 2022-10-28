function [data, info] = callSnapItResponse
%CallSnapIt gives an empty data for jsk_pcl_ros/CallSnapItResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_pcl_ros/CallSnapItResponse';
[data.Transformation, info.Transformation] = ros.internal.ros.messages.geometry_msgs.pose;
info.Transformation.MLdataType = 'struct';
info.MessageType = 'jsk_pcl_ros/CallSnapItResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'transformation';
info.MatPath{2} = 'transformation.position';
info.MatPath{3} = 'transformation.position.x';
info.MatPath{4} = 'transformation.position.y';
info.MatPath{5} = 'transformation.position.z';
info.MatPath{6} = 'transformation.orientation';
info.MatPath{7} = 'transformation.orientation.x';
info.MatPath{8} = 'transformation.orientation.y';
info.MatPath{9} = 'transformation.orientation.z';
info.MatPath{10} = 'transformation.orientation.w';
