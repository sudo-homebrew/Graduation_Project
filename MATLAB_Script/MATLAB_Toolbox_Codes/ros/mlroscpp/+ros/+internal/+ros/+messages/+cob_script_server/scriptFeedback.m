function [data, info] = scriptFeedback
%ScriptFeedback gives an empty data for cob_script_server/ScriptFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_script_server/ScriptFeedback';
info.MessageType = 'cob_script_server/ScriptFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
