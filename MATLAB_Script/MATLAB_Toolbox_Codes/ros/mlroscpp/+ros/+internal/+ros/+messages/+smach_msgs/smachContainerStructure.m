function [data, info] = smachContainerStructure
%SmachContainerStructure gives an empty data for smach_msgs/SmachContainerStructure

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'smach_msgs/SmachContainerStructure';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Path, info.Path] = ros.internal.ros.messages.ros.char('string',0);
[data.Children, info.Children] = ros.internal.ros.messages.ros.char('string',NaN);
[data.InternalOutcomes, info.InternalOutcomes] = ros.internal.ros.messages.ros.char('string',NaN);
[data.OutcomesFrom, info.OutcomesFrom] = ros.internal.ros.messages.ros.char('string',NaN);
[data.OutcomesTo, info.OutcomesTo] = ros.internal.ros.messages.ros.char('string',NaN);
[data.ContainerOutcomes, info.ContainerOutcomes] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'smach_msgs/SmachContainerStructure';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'path';
info.MatPath{8} = 'children';
info.MatPath{9} = 'internal_outcomes';
info.MatPath{10} = 'outcomes_from';
info.MatPath{11} = 'outcomes_to';
info.MatPath{12} = 'container_outcomes';
