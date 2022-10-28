function [data, info] = fileOpenResponse
%FileOpen gives an empty data for mavros_msgs/FileOpenResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/FileOpenResponse';
[data.Size, info.Size] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.RErrno, info.RErrno] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'mavros_msgs/FileOpenResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'size';
info.MatPath{2} = 'success';
info.MatPath{3} = 'r_errno';
