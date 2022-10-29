function [data, info] = goalID
%GoalID gives an empty data for actionlib_msgs/GoalID

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'actionlib_msgs/GoalID';
[data.Stamp, info.Stamp] = ros.internal.ros.messages.ros.time;
info.Stamp.MLdataType = 'struct';
[data.Id, info.Id] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'actionlib_msgs/GoalID';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'stamp';
info.MatPath{2} = 'stamp.sec';
info.MatPath{3} = 'stamp.nsec';
info.MatPath{4} = 'id';
