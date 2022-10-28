function [data, info] = state
%State gives an empty data for robotnik_msgs/State

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/State';
[data.INITSTATE, info.INITSTATE] = ros.internal.ros.messages.ros.default_type('int32',1, 100);
[data.STANDBYSTATE, info.STANDBYSTATE] = ros.internal.ros.messages.ros.default_type('int32',1, 200);
[data.READYSTATE, info.READYSTATE] = ros.internal.ros.messages.ros.default_type('int32',1, 300);
[data.EMERGENCYSTATE, info.EMERGENCYSTATE] = ros.internal.ros.messages.ros.default_type('int32',1, 400);
[data.FAILURESTATE, info.FAILURESTATE] = ros.internal.ros.messages.ros.default_type('int32',1, 500);
[data.SHUTDOWNSTATE, info.SHUTDOWNSTATE] = ros.internal.ros.messages.ros.default_type('int32',1, 600);
[data.UNKOWNSTATE, info.UNKOWNSTATE] = ros.internal.ros.messages.ros.default_type('int32',1, 700);
[data.State_, info.State_] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.DesiredFreq, info.DesiredFreq] = ros.internal.ros.messages.ros.default_type('single',1);
[data.RealFreq, info.RealFreq] = ros.internal.ros.messages.ros.default_type('single',1);
[data.StateDescription, info.StateDescription] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'robotnik_msgs/State';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'INIT_STATE';
info.MatPath{2} = 'STANDBY_STATE';
info.MatPath{3} = 'READY_STATE';
info.MatPath{4} = 'EMERGENCY_STATE';
info.MatPath{5} = 'FAILURE_STATE';
info.MatPath{6} = 'SHUTDOWN_STATE';
info.MatPath{7} = 'UNKOWN_STATE';
info.MatPath{8} = 'state';
info.MatPath{9} = 'desired_freq';
info.MatPath{10} = 'real_freq';
info.MatPath{11} = 'state_description';
