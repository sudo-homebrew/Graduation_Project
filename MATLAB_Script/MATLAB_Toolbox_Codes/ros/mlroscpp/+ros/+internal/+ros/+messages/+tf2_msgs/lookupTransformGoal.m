function [data, info] = lookupTransformGoal
%LookupTransformGoal gives an empty data for tf2_msgs/LookupTransformGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'tf2_msgs/LookupTransformGoal';
[data.TargetFrame, info.TargetFrame] = ros.internal.ros.messages.ros.char('string',0);
[data.SourceFrame, info.SourceFrame] = ros.internal.ros.messages.ros.char('string',0);
[data.SourceTime, info.SourceTime] = ros.internal.ros.messages.ros.time;
info.SourceTime.MLdataType = 'struct';
[data.Timeout, info.Timeout] = ros.internal.ros.messages.ros.duration;
info.Timeout.MLdataType = 'struct';
[data.TargetTime, info.TargetTime] = ros.internal.ros.messages.ros.time;
info.TargetTime.MLdataType = 'struct';
[data.FixedFrame, info.FixedFrame] = ros.internal.ros.messages.ros.char('string',0);
[data.Advanced, info.Advanced] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'tf2_msgs/LookupTransformGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,13);
info.MatPath{1} = 'target_frame';
info.MatPath{2} = 'source_frame';
info.MatPath{3} = 'source_time';
info.MatPath{4} = 'source_time.sec';
info.MatPath{5} = 'source_time.nsec';
info.MatPath{6} = 'timeout';
info.MatPath{7} = 'timeout.sec';
info.MatPath{8} = 'timeout.nsec';
info.MatPath{9} = 'target_time';
info.MatPath{10} = 'target_time.sec';
info.MatPath{11} = 'target_time.nsec';
info.MatPath{12} = 'fixed_frame';
info.MatPath{13} = 'advanced';
