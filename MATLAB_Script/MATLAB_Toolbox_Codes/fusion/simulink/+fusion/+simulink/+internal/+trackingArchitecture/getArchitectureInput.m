function archInputs = getArchitectureInput(summary,nodeSeqNum)
%   Find architecture inputs for node.
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2021 The MathWorks, Inc.

archInputs = [];
if ~isempty(summary.ArchitectureInputs{nodeSeqNum})
    archInputs = str2double(strsplit(summary.ArchitectureInputs{nodeSeqNum}));
end
end