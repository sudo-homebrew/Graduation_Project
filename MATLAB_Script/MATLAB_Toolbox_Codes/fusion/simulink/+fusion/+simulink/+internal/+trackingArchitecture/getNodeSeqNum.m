function seqNum = getNodeSeqNum(nodes,index)
%   Find node sequence number from node index.
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2021 The MathWorks, Inc.

seqNum = find(cellfun(@(x)x.Index == index,nodes));
end