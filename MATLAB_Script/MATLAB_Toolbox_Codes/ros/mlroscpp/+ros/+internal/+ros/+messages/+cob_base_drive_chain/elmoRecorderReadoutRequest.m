function [data, info] = elmoRecorderReadoutRequest
%ElmoRecorderReadout gives an empty data for cob_base_drive_chain/ElmoRecorderReadoutRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_base_drive_chain/ElmoRecorderReadoutRequest';
[data.Subindex, info.Subindex] = ros.internal.ros.messages.ros.default_type('int64',1);
[data.Fileprefix, info.Fileprefix] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'cob_base_drive_chain/ElmoRecorderReadoutRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'subindex';
info.MatPath{2} = 'fileprefix';
