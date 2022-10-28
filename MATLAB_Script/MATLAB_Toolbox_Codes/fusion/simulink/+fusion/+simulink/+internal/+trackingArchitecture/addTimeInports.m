function addTimeInports(mdl,nodes,hblocks)
%   Add time inports in the model.
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2021 The MathWorks, Inc.

for i = 1:numel(nodes)
    if isequal(i,1)
        % Add a new time input port
        timePort = add_block('simulink/Sources/In1',strcat(mdl,'/Prediction Time'));
    else
        %Duplicate existing time port
        timePort = add_block([mdl,'/Prediction Time'],strcat(mdl,'/Prediction Time',num2str(nodes{i}.Index)),'CopyOption','duplicate');
    end
    fusion.simulink.internal.trackingArchitecture.connectBlocks(mdl, timePort, 1, hblocks(i), 2);
    fusion.simulink.internal.trackingArchitecture.resizeBlock(timePort,15,20);
    nodePos = get_param(hblocks(i),'position'); %Connected nodes position
    xOffset = 80;
    yOffset = 35;
    xpos = nodePos(1) - xOffset;
    ypos = nodePos(4) - yOffset;
    fusion.simulink.internal.trackingArchitecture.positionBlock(timePort,xpos,ypos);
end
end