function flag = isConnectionDelayed(delayedEdges,currentFuser,connectedFuser)
%   Find if a connection is delayed between two fuser nodes.
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2021 The MathWorks, Inc.

flag = any(delayedEdges(delayedEdges(:,1) == currentFuser,2) == connectedFuser);
end