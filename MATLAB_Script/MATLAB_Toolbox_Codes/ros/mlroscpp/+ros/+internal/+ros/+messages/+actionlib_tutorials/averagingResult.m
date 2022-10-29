function [data, info] = averagingResult
%AveragingResult gives an empty data for actionlib_tutorials/AveragingResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'actionlib_tutorials/AveragingResult';
[data.Mean, info.Mean] = ros.internal.ros.messages.ros.default_type('single',1);
[data.StdDev, info.StdDev] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'actionlib_tutorials/AveragingResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'mean';
info.MatPath{2} = 'std_dev';
