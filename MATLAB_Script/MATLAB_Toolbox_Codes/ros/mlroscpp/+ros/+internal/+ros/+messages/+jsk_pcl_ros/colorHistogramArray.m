function [data, info] = colorHistogramArray
%ColorHistogramArray gives an empty data for jsk_pcl_ros/ColorHistogramArray

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_pcl_ros/ColorHistogramArray';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Histograms, info.Histograms] = ros.internal.ros.messages.jsk_pcl_ros.colorHistogram;
info.Histograms.MLdataType = 'struct';
info.Histograms.MaxLen = NaN;
info.Histograms.MinLen = 0;
data.Histograms = data.Histograms([],1);
info.MessageType = 'jsk_pcl_ros/ColorHistogramArray';
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
info.MatPath{7} = 'histograms';
info.MatPath{8} = 'histograms.header';
info.MatPath{9} = 'histograms.header.seq';
info.MatPath{10} = 'histograms.header.stamp';
info.MatPath{11} = 'histograms.header.stamp.sec';
info.MatPath{12} = 'histograms.header.stamp.nsec';
info.MatPath{13} = 'histograms.header.frame_id';
info.MatPath{14} = 'histograms.histogram';
