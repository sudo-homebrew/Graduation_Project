function [data, info] = setModeRequest
%SetMode gives an empty data for ethzasl_icp_mapper/SetModeRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ethzasl_icp_mapper/SetModeRequest';
[data.Localize, info.Localize] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Map, info.Map] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.ApplyChange, info.ApplyChange] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'ethzasl_icp_mapper/SetModeRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'localize';
info.MatPath{2} = 'map';
info.MatPath{3} = 'applyChange';
