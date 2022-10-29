function [data, info] = sendupdate
%sendupdate gives an empty data for sr_robot_msgs/sendupdate

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/sendupdate';
[data.SendupdateLength, info.SendupdateLength] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.SendupdateList, info.SendupdateList] = ros.internal.ros.messages.sr_robot_msgs.joint;
info.SendupdateList.MLdataType = 'struct';
info.SendupdateList.MaxLen = NaN;
info.SendupdateList.MinLen = 0;
data.SendupdateList = data.SendupdateList([],1);
info.MessageType = 'sr_robot_msgs/sendupdate';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'sendupdate_length';
info.MatPath{2} = 'sendupdate_list';
info.MatPath{3} = 'sendupdate_list.joint_name';
info.MatPath{4} = 'sendupdate_list.joint_position';
info.MatPath{5} = 'sendupdate_list.joint_target';
info.MatPath{6} = 'sendupdate_list.joint_torque';
info.MatPath{7} = 'sendupdate_list.joint_temperature';
info.MatPath{8} = 'sendupdate_list.joint_current';
info.MatPath{9} = 'sendupdate_list.error_flag';
