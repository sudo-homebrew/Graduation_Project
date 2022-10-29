function [data, info] = joy
%Joy gives an empty data for clearpath_base/Joy

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'clearpath_base/Joy';
[data.Axes, info.Axes] = ros.internal.ros.messages.ros.default_type('single',NaN);
[data.Buttons, info.Buttons] = ros.internal.ros.messages.ros.default_type('int32',NaN);
info.MessageType = 'clearpath_base/Joy';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'axes';
info.MatPath{2} = 'buttons';
