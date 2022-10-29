function [data, info] = personStamped
%PersonStamped gives an empty data for people_msgs/PersonStamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'people_msgs/PersonStamped';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Person, info.Person] = ros.internal.ros.messages.people_msgs.person;
info.Person.MLdataType = 'struct';
info.MessageType = 'people_msgs/PersonStamped';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,19);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'person';
info.MatPath{8} = 'person.name';
info.MatPath{9} = 'person.position';
info.MatPath{10} = 'person.position.x';
info.MatPath{11} = 'person.position.y';
info.MatPath{12} = 'person.position.z';
info.MatPath{13} = 'person.velocity';
info.MatPath{14} = 'person.velocity.x';
info.MatPath{15} = 'person.velocity.y';
info.MatPath{16} = 'person.velocity.z';
info.MatPath{17} = 'person.reliability';
info.MatPath{18} = 'person.tagnames';
info.MatPath{19} = 'person.tags';
