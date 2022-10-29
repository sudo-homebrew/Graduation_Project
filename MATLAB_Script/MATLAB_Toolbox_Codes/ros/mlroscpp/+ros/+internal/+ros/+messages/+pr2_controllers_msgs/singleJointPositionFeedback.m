function [data, info] = singleJointPositionFeedback
%SingleJointPositionFeedback gives an empty data for pr2_controllers_msgs/SingleJointPositionFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_controllers_msgs/SingleJointPositionFeedback';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Position, info.Position] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Velocity, info.Velocity] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Error, info.Error] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'pr2_controllers_msgs/SingleJointPositionFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'position';
info.MatPath{8} = 'velocity';
info.MatPath{9} = 'error';
