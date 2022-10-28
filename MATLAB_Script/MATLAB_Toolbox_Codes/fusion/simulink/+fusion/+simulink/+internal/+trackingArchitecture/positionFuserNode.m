function positionFuserNode(blkHndl,nodePos,verticalGap,horizontalGap)
%   Position fuser node in the model.
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2021 The MathWorks, Inc.

xPos = horizontalGap * (nodePos.XPosition-2);
yPos = nodePos.YPosition*verticalGap;
fusion.simulink.internal.trackingArchitecture.positionBlock(blkHndl,xPos,yPos);
end