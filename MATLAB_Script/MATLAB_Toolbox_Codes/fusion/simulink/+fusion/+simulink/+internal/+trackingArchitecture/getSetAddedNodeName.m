function nodeName = getSetAddedNodeName(nodes,nodeNames,index,name)
%   Get and set node names based on node index.
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2021 The MathWorks, Inc.

seqNum = fusion.simulink.internal.trackingArchitecture.getNodeSeqNum(nodes,index);
if isequal(nargin,4)
    %Set node name.
    nodeNames{seqNum} = name;
    nodeName = nodeNames;
else
    %Get node name.
    nodeName = nodeNames{seqNum};
end
end