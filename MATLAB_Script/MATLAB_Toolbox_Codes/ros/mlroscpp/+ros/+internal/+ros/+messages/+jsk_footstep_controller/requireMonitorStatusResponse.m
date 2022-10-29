function [data, info] = requireMonitorStatusResponse
%RequireMonitorStatus gives an empty data for jsk_footstep_controller/RequireMonitorStatusResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_footstep_controller/RequireMonitorStatusResponse';
[data.Go, info.Go] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'jsk_footstep_controller/RequireMonitorStatusResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'go';
