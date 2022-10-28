function [data, info] = oCS2FC
%OCS2FC gives an empty data for jsk_network_tools/OCS2FC

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_network_tools/OCS2FC';
[data.JointAngles, info.JointAngles] = ros.internal.ros.messages.ros.default_type('uint8',32);
[data.StartImpedance, info.StartImpedance] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Stop, info.Stop] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'jsk_network_tools/OCS2FC';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'joint_angles';
info.MatPath{2} = 'start_impedance';
info.MatPath{3} = 'stop';
