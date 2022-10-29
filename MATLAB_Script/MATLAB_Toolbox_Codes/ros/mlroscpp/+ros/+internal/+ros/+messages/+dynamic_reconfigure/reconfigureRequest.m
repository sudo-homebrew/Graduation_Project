function [data, info] = reconfigureRequest
%Reconfigure gives an empty data for dynamic_reconfigure/ReconfigureRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'dynamic_reconfigure/ReconfigureRequest';
[data.Config, info.Config] = ros.internal.ros.messages.dynamic_reconfigure.config;
info.Config.MLdataType = 'struct';
info.MessageType = 'dynamic_reconfigure/ReconfigureRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,18);
info.MatPath{1} = 'config';
info.MatPath{2} = 'config.bools';
info.MatPath{3} = 'config.bools.name';
info.MatPath{4} = 'config.bools.value';
info.MatPath{5} = 'config.ints';
info.MatPath{6} = 'config.ints.name';
info.MatPath{7} = 'config.ints.value';
info.MatPath{8} = 'config.strs';
info.MatPath{9} = 'config.strs.name';
info.MatPath{10} = 'config.strs.value';
info.MatPath{11} = 'config.doubles';
info.MatPath{12} = 'config.doubles.name';
info.MatPath{13} = 'config.doubles.value';
info.MatPath{14} = 'config.groups';
info.MatPath{15} = 'config.groups.name';
info.MatPath{16} = 'config.groups.state';
info.MatPath{17} = 'config.groups.id';
info.MatPath{18} = 'config.groups.parent';
