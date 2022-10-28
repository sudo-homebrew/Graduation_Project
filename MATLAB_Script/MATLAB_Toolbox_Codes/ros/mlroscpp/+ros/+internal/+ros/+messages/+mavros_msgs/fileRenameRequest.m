function [data, info] = fileRenameRequest
%FileRename gives an empty data for mavros_msgs/FileRenameRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/FileRenameRequest';
[data.OldPath, info.OldPath] = ros.internal.ros.messages.ros.char('string',0);
[data.NewPath, info.NewPath] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'mavros_msgs/FileRenameRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'old_path';
info.MatPath{2} = 'new_path';
