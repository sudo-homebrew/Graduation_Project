function [data, info] = elmoRecorderConfigRequest
%ElmoRecorderConfig gives an empty data for cob_base_drive_chain/ElmoRecorderConfigRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_base_drive_chain/ElmoRecorderConfigRequest';
[data.Recordinggap, info.Recordinggap] = ros.internal.ros.messages.ros.default_type('int64',1);
info.MessageType = 'cob_base_drive_chain/ElmoRecorderConfigRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'recordinggap';
