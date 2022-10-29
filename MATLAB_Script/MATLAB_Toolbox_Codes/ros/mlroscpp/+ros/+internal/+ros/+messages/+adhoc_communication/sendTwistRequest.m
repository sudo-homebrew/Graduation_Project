function [data, info] = sendTwistRequest
%SendTwist gives an empty data for adhoc_communication/SendTwistRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'adhoc_communication/SendTwistRequest';
[data.DstRobot, info.DstRobot] = ros.internal.ros.messages.ros.char('string',0);
[data.Topic, info.Topic] = ros.internal.ros.messages.ros.char('string',0);
[data.Twist, info.Twist] = ros.internal.ros.messages.geometry_msgs.twist;
info.Twist.MLdataType = 'struct';
info.MessageType = 'adhoc_communication/SendTwistRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'dst_robot';
info.MatPath{2} = 'topic';
info.MatPath{3} = 'twist';
info.MatPath{4} = 'twist.linear';
info.MatPath{5} = 'twist.linear.x';
info.MatPath{6} = 'twist.linear.y';
info.MatPath{7} = 'twist.linear.z';
info.MatPath{8} = 'twist.angular';
info.MatPath{9} = 'twist.angular.x';
info.MatPath{10} = 'twist.angular.y';
info.MatPath{11} = 'twist.angular.z';
