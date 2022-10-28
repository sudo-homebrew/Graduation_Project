function [data, info] = commonFooter
%CommonFooter gives an empty data for applanix_msgs/CommonFooter

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/CommonFooter';
[data.END, info.END] = ros.internal.ros.messages.ros.char('string',0);
[data.END, info.END] = ros.internal.ros.messages.ros.char('string',1,'$#');
[data.Checksum, info.Checksum] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.End, info.End] = ros.internal.ros.messages.ros.default_type('uint8',2);
info.MessageType = 'applanix_msgs/CommonFooter';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'END';
info.MatPath{2} = 'checksum';
info.MatPath{3} = 'end';
