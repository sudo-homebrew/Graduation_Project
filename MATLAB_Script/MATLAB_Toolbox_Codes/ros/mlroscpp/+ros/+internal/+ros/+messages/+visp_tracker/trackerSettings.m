function [data, info] = trackerSettings
%TrackerSettings gives an empty data for visp_tracker/TrackerSettings

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'visp_tracker/TrackerSettings';
[data.AngleAppear, info.AngleAppear] = ros.internal.ros.messages.ros.default_type('double',1);
[data.AngleDisappear, info.AngleDisappear] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'visp_tracker/TrackerSettings';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'angle_appear';
info.MatPath{2} = 'angle_disappear';
