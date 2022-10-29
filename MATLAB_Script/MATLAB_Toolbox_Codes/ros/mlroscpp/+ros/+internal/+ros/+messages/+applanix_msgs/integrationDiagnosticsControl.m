function [data, info] = integrationDiagnosticsControl
%IntegrationDiagnosticsControl gives an empty data for applanix_msgs/IntegrationDiagnosticsControl

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/IntegrationDiagnosticsControl';
[data.Transaction, info.Transaction] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.OUTPUTIMUFRAME, info.OUTPUTIMUFRAME] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.OUTPUTUSERPARAM, info.OUTPUTUSERPARAM] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.OutputData, info.OutputData] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.UserRoll, info.UserRoll] = ros.internal.ros.messages.ros.default_type('single',1);
[data.UserPitch, info.UserPitch] = ros.internal.ros.messages.ros.default_type('single',1);
[data.UserHeading, info.UserHeading] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Reserved, info.Reserved] = ros.internal.ros.messages.ros.default_type('uint8',4);
info.MessageType = 'applanix_msgs/IntegrationDiagnosticsControl';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'transaction';
info.MatPath{2} = 'OUTPUT_IMU_FRAME';
info.MatPath{3} = 'OUTPUT_USER_PARAM';
info.MatPath{4} = 'output_data';
info.MatPath{5} = 'user_roll';
info.MatPath{6} = 'user_pitch';
info.MatPath{7} = 'user_heading';
info.MatPath{8} = 'reserved';
