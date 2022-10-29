function [data, info] = modelCoefficientsArray
%ModelCoefficientsArray gives an empty data for jsk_pcl_ros/ModelCoefficientsArray

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_pcl_ros/ModelCoefficientsArray';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Coefficients, info.Coefficients] = ros.internal.ros.messages.pcl_msgs.modelCoefficients;
info.Coefficients.MLdataType = 'struct';
info.Coefficients.MaxLen = NaN;
info.Coefficients.MinLen = 0;
data.Coefficients = data.Coefficients([],1);
info.MessageType = 'jsk_pcl_ros/ModelCoefficientsArray';
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
info.MatPath{7} = 'coefficients';
info.MatPath{8} = 'coefficients.header';
info.MatPath{9} = 'coefficients.header.seq';
info.MatPath{10} = 'coefficients.header.stamp';
info.MatPath{11} = 'coefficients.header.stamp.sec';
info.MatPath{12} = 'coefficients.header.stamp.nsec';
info.MatPath{13} = 'coefficients.header.frame_id';
info.MatPath{14} = 'coefficients.values';
