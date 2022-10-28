function h =  getNodeHandle(nodeIndex,nodeHandles,concathandles,nodes)
%   The function returns handle to the concatenation block if it exists
%   otherwise it returns handle to the tracker or fuser node.
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2021 The MathWorks, Inc.

seqNum = fusion.simulink.internal.trackingArchitecture.getNodeSeqNum(nodes,nodeIndex);
h = concathandles(seqNum);
if h
    return
end
h = nodeHandles(seqNum);
end