function [data, info] = person
%Person gives an empty data for people_msgs/Person

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'people_msgs/Person';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Position, info.Position] = ros.internal.ros.messages.geometry_msgs.point;
info.Position.MLdataType = 'struct';
[data.Velocity, info.Velocity] = ros.internal.ros.messages.geometry_msgs.point;
info.Velocity.MLdataType = 'struct';
[data.Reliability, info.Reliability] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Tagnames, info.Tagnames] = ros.internal.ros.messages.ros.char('string',NaN);
[data.Tags, info.Tags] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'people_msgs/Person';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'name';
info.MatPath{2} = 'position';
info.MatPath{3} = 'position.x';
info.MatPath{4} = 'position.y';
info.MatPath{5} = 'position.z';
info.MatPath{6} = 'velocity';
info.MatPath{7} = 'velocity.x';
info.MatPath{8} = 'velocity.y';
info.MatPath{9} = 'velocity.z';
info.MatPath{10} = 'reliability';
info.MatPath{11} = 'tagnames';
info.MatPath{12} = 'tags';
