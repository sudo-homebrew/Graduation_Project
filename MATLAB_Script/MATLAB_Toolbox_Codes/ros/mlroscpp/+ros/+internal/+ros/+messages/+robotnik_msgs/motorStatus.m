function [data, info] = motorStatus
%MotorStatus gives an empty data for robotnik_msgs/MotorStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/MotorStatus';
[data.State, info.State] = ros.internal.ros.messages.ros.char('string',0);
[data.Status, info.Status] = ros.internal.ros.messages.ros.char('string',0);
[data.Communicationstatus, info.Communicationstatus] = ros.internal.ros.messages.ros.char('string',0);
[data.Statusword, info.Statusword] = ros.internal.ros.messages.ros.char('string',0);
[data.Driveflags, info.Driveflags] = ros.internal.ros.messages.ros.char('string',0);
[data.Activestatusword, info.Activestatusword] = ros.internal.ros.messages.ros.char('string',NaN);
[data.Activedriveflags, info.Activedriveflags] = ros.internal.ros.messages.ros.char('string',NaN);
[data.Digitaloutputs, info.Digitaloutputs] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Digitalinputs, info.Digitalinputs] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Averagecurrent, info.Averagecurrent] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Analoginputs, info.Analoginputs] = ros.internal.ros.messages.ros.default_type('single',NaN);
info.MessageType = 'robotnik_msgs/MotorStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'state';
info.MatPath{2} = 'status';
info.MatPath{3} = 'communicationstatus';
info.MatPath{4} = 'statusword';
info.MatPath{5} = 'driveflags';
info.MatPath{6} = 'activestatusword';
info.MatPath{7} = 'activedriveflags';
info.MatPath{8} = 'digitaloutputs';
info.MatPath{9} = 'digitalinputs';
info.MatPath{10} = 'averagecurrent';
info.MatPath{11} = 'analoginputs';
