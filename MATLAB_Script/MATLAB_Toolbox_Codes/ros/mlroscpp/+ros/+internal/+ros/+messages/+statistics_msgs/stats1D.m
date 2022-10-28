function [data, info] = stats1D
%Stats1D gives an empty data for statistics_msgs/Stats1D

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'statistics_msgs/Stats1D';
[data.Min, info.Min] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Max, info.Max] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Mean, info.Mean] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Variance, info.Variance] = ros.internal.ros.messages.ros.default_type('double',1);
[data.N, info.N] = ros.internal.ros.messages.ros.default_type('int64',1);
info.MessageType = 'statistics_msgs/Stats1D';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'min';
info.MatPath{2} = 'max';
info.MatPath{3} = 'mean';
info.MatPath{4} = 'variance';
info.MatPath{5} = 'N';
