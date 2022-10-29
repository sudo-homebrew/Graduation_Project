function [data, info] = perceiveChemLabToolRequest
%PerceiveChemLabTool gives an empty data for dna_extraction_msgs/PerceiveChemLabToolRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'dna_extraction_msgs/PerceiveChemLabToolRequest';
[data.ToolName, info.ToolName] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'dna_extraction_msgs/PerceiveChemLabToolRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'tool_name';
