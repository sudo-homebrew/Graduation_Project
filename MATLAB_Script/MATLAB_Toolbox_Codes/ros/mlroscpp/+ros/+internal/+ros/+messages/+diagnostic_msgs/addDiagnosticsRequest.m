function [data, info] = addDiagnosticsRequest
%AddDiagnostics gives an empty data for diagnostic_msgs/AddDiagnosticsRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'diagnostic_msgs/AddDiagnosticsRequest';
[data.LoadNamespace, info.LoadNamespace] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'diagnostic_msgs/AddDiagnosticsRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'load_namespace';
