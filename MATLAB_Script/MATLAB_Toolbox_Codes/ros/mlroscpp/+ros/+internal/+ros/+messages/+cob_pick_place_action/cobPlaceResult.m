function [data, info] = cobPlaceResult
%CobPlaceResult gives an empty data for cob_pick_place_action/CobPlaceResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_pick_place_action/CobPlaceResult';
[data.Success, info.Success] = ros.internal.ros.messages.std_msgs.bool;
info.Success.MLdataType = 'struct';
[data.CobPickErrorString, info.CobPickErrorString] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'cob_pick_place_action/CobPlaceResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'success';
info.MatPath{2} = 'success.data';
info.MatPath{3} = 'cob_pick_error_string';
