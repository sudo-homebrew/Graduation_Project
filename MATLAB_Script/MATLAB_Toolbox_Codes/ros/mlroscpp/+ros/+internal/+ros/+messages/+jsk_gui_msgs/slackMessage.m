function [data, info] = slackMessage
%SlackMessage gives an empty data for jsk_gui_msgs/SlackMessage

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_gui_msgs/SlackMessage';
[data.Channel, info.Channel] = ros.internal.ros.messages.ros.char('string',0);
[data.Text, info.Text] = ros.internal.ros.messages.ros.char('string',0);
[data.Image, info.Image] = ros.internal.ros.messages.sensor_msgs.image;
info.Image.MLdataType = 'struct';
info.MessageType = 'jsk_gui_msgs/SlackMessage';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,15);
info.MatPath{1} = 'channel';
info.MatPath{2} = 'text';
info.MatPath{3} = 'image';
info.MatPath{4} = 'image.header';
info.MatPath{5} = 'image.header.seq';
info.MatPath{6} = 'image.header.stamp';
info.MatPath{7} = 'image.header.stamp.sec';
info.MatPath{8} = 'image.header.stamp.nsec';
info.MatPath{9} = 'image.header.frame_id';
info.MatPath{10} = 'image.height';
info.MatPath{11} = 'image.width';
info.MatPath{12} = 'image.encoding';
info.MatPath{13} = 'image.is_bigendian';
info.MatPath{14} = 'image.step';
info.MatPath{15} = 'image.data';
