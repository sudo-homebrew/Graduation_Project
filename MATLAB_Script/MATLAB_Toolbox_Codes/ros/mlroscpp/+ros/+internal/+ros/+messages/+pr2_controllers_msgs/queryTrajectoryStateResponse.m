function [data, info] = queryTrajectoryStateResponse
%QueryTrajectoryState gives an empty data for pr2_controllers_msgs/QueryTrajectoryStateResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_controllers_msgs/QueryTrajectoryStateResponse';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',NaN);
[data.Position, info.Position] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.Velocity, info.Velocity] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.Acceleration, info.Acceleration] = ros.internal.ros.messages.ros.default_type('double',NaN);
info.MessageType = 'pr2_controllers_msgs/QueryTrajectoryStateResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'name';
info.MatPath{2} = 'position';
info.MatPath{3} = 'velocity';
info.MatPath{4} = 'acceleration';
