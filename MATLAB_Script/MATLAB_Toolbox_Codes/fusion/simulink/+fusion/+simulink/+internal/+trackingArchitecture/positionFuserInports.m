function positionFuserInports(nodeHndl,inputs,archInput)
%   Position fuser inports in a Simulink model.
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2021 The MathWorks, Inc.

numArchInputs = numel(archInput);
if isequal(inputs,archInput) && numArchInputs > 0
    %Fuser node only has architecture inputs.
    fusion.simulink.internal.trackingArchitecture.positionInports(nodeHndl,numArchInputs);
else
    %Fuser node has both architecture input and inputs from other nodes
    %within the architecture.
    archInputIndices = find(ismember(inputs,archInput));
    blkPos  = get_param(nodeHndl,'position');
    prtCon = get_param(nodeHndl,'PortConnectivity');
    xOffset = 80;
    xPos = blkPos(1) - xOffset;
    numNonArchInputs = numel(inputs) - numArchInputs;
    verticalGap = 35;
    yOffset = numNonArchInputs * verticalGap ;
    yPos = blkPos(2)+ yOffset;
    for i = 1:numArchInputs
        fusion.simulink.internal.trackingArchitecture.positionBlock(prtCon(archInputIndices(i)).SrcBlock,xPos,yPos);
        yPos = yPos+verticalGap;
    end
end
end