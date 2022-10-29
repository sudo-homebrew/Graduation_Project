function [data, info] = cO2SourceVector
%CO2SourceVector gives an empty data for stdr_msgs/CO2SourceVector

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'stdr_msgs/CO2SourceVector';
[data.Co2Sources, info.Co2Sources] = ros.internal.ros.messages.stdr_msgs.cO2Source;
info.Co2Sources.MLdataType = 'struct';
info.Co2Sources.MaxLen = NaN;
info.Co2Sources.MinLen = 0;
data.Co2Sources = data.Co2Sources([],1);
info.MessageType = 'stdr_msgs/CO2SourceVector';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'co2_sources';
info.MatPath{2} = 'co2_sources.id';
info.MatPath{3} = 'co2_sources.ppm';
info.MatPath{4} = 'co2_sources.pose';
info.MatPath{5} = 'co2_sources.pose.x';
info.MatPath{6} = 'co2_sources.pose.y';
info.MatPath{7} = 'co2_sources.pose.theta';
