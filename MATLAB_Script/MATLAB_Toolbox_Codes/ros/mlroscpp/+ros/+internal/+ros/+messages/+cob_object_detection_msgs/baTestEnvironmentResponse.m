function [data, info] = baTestEnvironmentResponse
%BaTestEnvironment gives an empty data for cob_object_detection_msgs/BaTestEnvironmentResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_object_detection_msgs/BaTestEnvironmentResponse';
[data.MeanError, info.MeanError] = ros.internal.ros.messages.ros.default_type('single',1);
[data.StdDev, info.StdDev] = ros.internal.ros.messages.ros.default_type('single',1);
[data.MinError, info.MinError] = ros.internal.ros.messages.ros.default_type('single',1);
[data.MaxError, info.MaxError] = ros.internal.ros.messages.ros.default_type('single',1);
[data.RunsCount, info.RunsCount] = ros.internal.ros.messages.ros.default_type('single',1);
[data.RunsSum, info.RunsSum] = ros.internal.ros.messages.ros.default_type('single',1);
[data.RunsSum2, info.RunsSum2] = ros.internal.ros.messages.ros.default_type('single',1);
[data.TimeDuration, info.TimeDuration] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Observations, info.Observations] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.FalseMatchings, info.FalseMatchings] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Result, info.Result] = ros.internal.ros.messages.std_msgs.string;
info.Result.MLdataType = 'struct';
info.MessageType = 'cob_object_detection_msgs/BaTestEnvironmentResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'mean_error';
info.MatPath{2} = 'std_dev';
info.MatPath{3} = 'min_error';
info.MatPath{4} = 'max_error';
info.MatPath{5} = 'runs_count';
info.MatPath{6} = 'runs_sum';
info.MatPath{7} = 'runs_sum2';
info.MatPath{8} = 'time_duration';
info.MatPath{9} = 'observations';
info.MatPath{10} = 'false_matchings';
info.MatPath{11} = 'result';
info.MatPath{12} = 'result.data';
