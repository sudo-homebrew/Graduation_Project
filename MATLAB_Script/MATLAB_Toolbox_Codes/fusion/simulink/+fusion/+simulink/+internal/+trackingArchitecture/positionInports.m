function positionInports(nodeHndl,numInputs)
%   Position inports when exporting a trackingArchitecture in such a way that
%   ports are not overlapping.
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2021 The MathWorks, Inc.

blkPos  = get_param(nodeHndl,'position');
prtCon = get_param(nodeHndl,'PortConnectivity');
xOffset = 80;
xPos = blkPos(1) - xOffset;
verticalGap = 35;
switch numInputs
    case 1
        yOffset = 15;
        yPos = blkPos(2) + yOffset;
        fusion.simulink.internal.trackingArchitecture.positionBlock(prtCon(1).SrcBlock,xPos,yPos);
    case 2
        %Port 1
        yOffset = 10;
        yPos = blkPos(2) + yOffset;
        fusion.simulink.internal.trackingArchitecture.positionBlock(prtCon(1).SrcBlock,xPos,yPos);
        %Port 2
        yOffset = yOffset + verticalGap;
        yPos = blkPos(2) + yOffset;
        fusion.simulink.internal.trackingArchitecture.positionBlock(prtCon(2).SrcBlock,xPos,yPos);
    otherwise
        blkHeight = blkPos(4)-blkPos(2);
        yOffset =  blkPos(2)- (numInputs* verticalGap - blkHeight)/2;
        yPos = yOffset;
        xPos = xPos - numInputs*5;
        for i = 1: numInputs
            fusion.simulink.internal.trackingArchitecture.positionBlock(prtCon(i).SrcBlock,xPos,yPos);
            yPos = yPos+verticalGap;
        end
end
end