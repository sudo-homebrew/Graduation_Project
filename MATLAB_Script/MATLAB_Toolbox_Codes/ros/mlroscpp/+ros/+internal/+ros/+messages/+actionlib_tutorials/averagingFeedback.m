function [data, info] = averagingFeedback
%AveragingFeedback gives an empty data for actionlib_tutorials/AveragingFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'actionlib_tutorials/AveragingFeedback';
[data.Sample, info.Sample] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Data, info.Data] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Mean, info.Mean] = ros.internal.ros.messages.ros.default_type('single',1);
[data.StdDev, info.StdDev] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'actionlib_tutorials/AveragingFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'sample';
info.MatPath{2} = 'data';
info.MatPath{3} = 'mean';
info.MatPath{4} = 'std_dev';
