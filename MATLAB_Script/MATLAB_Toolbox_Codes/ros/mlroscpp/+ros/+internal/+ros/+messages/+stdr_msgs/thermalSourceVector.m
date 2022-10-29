function [data, info] = thermalSourceVector
%ThermalSourceVector gives an empty data for stdr_msgs/ThermalSourceVector

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'stdr_msgs/ThermalSourceVector';
[data.ThermalSources, info.ThermalSources] = ros.internal.ros.messages.stdr_msgs.thermalSource;
info.ThermalSources.MLdataType = 'struct';
info.ThermalSources.MaxLen = NaN;
info.ThermalSources.MinLen = 0;
data.ThermalSources = data.ThermalSources([],1);
info.MessageType = 'stdr_msgs/ThermalSourceVector';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'thermal_sources';
info.MatPath{2} = 'thermal_sources.id';
info.MatPath{3} = 'thermal_sources.degrees';
info.MatPath{4} = 'thermal_sources.pose';
info.MatPath{5} = 'thermal_sources.pose.x';
info.MatPath{6} = 'thermal_sources.pose.y';
info.MatPath{7} = 'thermal_sources.pose.theta';
