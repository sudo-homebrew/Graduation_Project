function [data, info] = fileListRequest
%FileList gives an empty data for mavros_msgs/FileListRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/FileListRequest';
[data.DirPath, info.DirPath] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'mavros_msgs/FileListRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'dir_path';
