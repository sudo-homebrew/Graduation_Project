function [data, info] = allowedCollisionMatrix
%AllowedCollisionMatrix gives an empty data for moveit_msgs/AllowedCollisionMatrix

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/AllowedCollisionMatrix';
[data.EntryNames, info.EntryNames] = ros.internal.ros.messages.ros.char('string',NaN);
[data.EntryValues, info.EntryValues] = ros.internal.ros.messages.moveit_msgs.allowedCollisionEntry;
info.EntryValues.MLdataType = 'struct';
info.EntryValues.MaxLen = NaN;
info.EntryValues.MinLen = 0;
data.EntryValues = data.EntryValues([],1);
[data.DefaultEntryNames, info.DefaultEntryNames] = ros.internal.ros.messages.ros.char('string',NaN);
[data.DefaultEntryValues, info.DefaultEntryValues] = ros.internal.ros.messages.ros.default_type('logical',NaN);
info.MessageType = 'moveit_msgs/AllowedCollisionMatrix';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'entry_names';
info.MatPath{2} = 'entry_values';
info.MatPath{3} = 'entry_values.enabled';
info.MatPath{4} = 'default_entry_names';
info.MatPath{5} = 'default_entry_values';
