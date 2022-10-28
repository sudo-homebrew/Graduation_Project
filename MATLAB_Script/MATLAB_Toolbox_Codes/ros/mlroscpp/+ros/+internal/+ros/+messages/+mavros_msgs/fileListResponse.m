function [data, info] = fileListResponse
%FileList gives an empty data for mavros_msgs/FileListResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/FileListResponse';
[data.List, info.List] = ros.internal.ros.messages.mavros_msgs.fileEntry;
info.List.MLdataType = 'struct';
info.List.MaxLen = NaN;
info.List.MinLen = 0;
data.List = data.List([],1);
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.RErrno, info.RErrno] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'mavros_msgs/FileListResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'list';
info.MatPath{2} = 'list.TYPE_FILE';
info.MatPath{3} = 'list.TYPE_DIRECTORY';
info.MatPath{4} = 'list.name';
info.MatPath{5} = 'list.type';
info.MatPath{6} = 'list.size';
info.MatPath{7} = 'success';
info.MatPath{8} = 'r_errno';
