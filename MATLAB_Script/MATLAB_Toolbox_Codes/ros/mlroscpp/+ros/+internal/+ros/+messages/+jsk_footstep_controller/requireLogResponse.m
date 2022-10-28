function [data, info] = requireLogResponse
%RequireLog gives an empty data for jsk_footstep_controller/RequireLogResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_footstep_controller/RequireLogResponse';
[data.Sexp, info.Sexp] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'jsk_footstep_controller/RequireLogResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'sexp';
