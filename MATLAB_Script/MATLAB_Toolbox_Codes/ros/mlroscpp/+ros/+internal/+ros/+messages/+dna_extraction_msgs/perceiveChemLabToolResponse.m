function [data, info] = perceiveChemLabToolResponse
%PerceiveChemLabTool gives an empty data for dna_extraction_msgs/PerceiveChemLabToolResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'dna_extraction_msgs/PerceiveChemLabToolResponse';
[data.ToolPose, info.ToolPose] = ros.internal.ros.messages.geometry_msgs.transformStamped;
info.ToolPose.MLdataType = 'struct';
info.MessageType = 'dna_extraction_msgs/PerceiveChemLabToolResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,18);
info.MatPath{1} = 'tool_pose';
info.MatPath{2} = 'tool_pose.header';
info.MatPath{3} = 'tool_pose.header.seq';
info.MatPath{4} = 'tool_pose.header.stamp';
info.MatPath{5} = 'tool_pose.header.stamp.sec';
info.MatPath{6} = 'tool_pose.header.stamp.nsec';
info.MatPath{7} = 'tool_pose.header.frame_id';
info.MatPath{8} = 'tool_pose.child_frame_id';
info.MatPath{9} = 'tool_pose.transform';
info.MatPath{10} = 'tool_pose.transform.translation';
info.MatPath{11} = 'tool_pose.transform.translation.x';
info.MatPath{12} = 'tool_pose.transform.translation.y';
info.MatPath{13} = 'tool_pose.transform.translation.z';
info.MatPath{14} = 'tool_pose.transform.rotation';
info.MatPath{15} = 'tool_pose.transform.rotation.x';
info.MatPath{16} = 'tool_pose.transform.rotation.y';
info.MatPath{17} = 'tool_pose.transform.rotation.z';
info.MatPath{18} = 'tool_pose.transform.rotation.w';
