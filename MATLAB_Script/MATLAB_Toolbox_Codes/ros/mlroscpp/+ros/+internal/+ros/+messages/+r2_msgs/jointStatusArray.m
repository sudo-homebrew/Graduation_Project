function [data, info] = jointStatusArray
%JointStatusArray gives an empty data for r2_msgs/JointStatusArray

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'r2_msgs/JointStatusArray';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Status, info.Status] = ros.internal.ros.messages.r2_msgs.jointStatus;
info.Status.MLdataType = 'struct';
info.Status.MaxLen = NaN;
info.Status.MinLen = 0;
data.Status = data.Status([],1);
info.MessageType = 'r2_msgs/JointStatusArray';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,16);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'status';
info.MatPath{8} = 'status.publisher';
info.MatPath{9} = 'status.joint';
info.MatPath{10} = 'status.registerValue';
info.MatPath{11} = 'status.bridgeEnabled';
info.MatPath{12} = 'status.motorEnabled';
info.MatPath{13} = 'status.brakeReleased';
info.MatPath{14} = 'status.motorPowerDetected';
info.MatPath{15} = 'status.embeddedMotCom';
info.MatPath{16} = 'status.jointFaulted';
