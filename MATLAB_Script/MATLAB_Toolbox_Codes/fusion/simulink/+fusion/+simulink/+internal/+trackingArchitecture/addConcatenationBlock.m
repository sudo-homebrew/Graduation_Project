function blkHndl = addConcatenationBlock(mdl,name,nodeHndl,numSources)
%   Add a concatenation block and return the handle.
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2021 The MathWorks, Inc.

if startsWith(name,'F')
    %Add a track concatenation block
    blockPath = 'trackingutilitieslib/Track Concatenation';
    %Block name in format [F1:Track Concatenation]
    blockName = strcat(extractBefore(name,':'),':Track Concatenation');
else
    %Add a detection concatenation block
    blockPath = 'trackingutilitieslib/Detection Concatenation';
    %Block name in format [T1:Detection Concatenation]
    blockName = strcat(extractBefore(name,':'),':Detection Concatenation');
end
blkHndl = add_block(blockPath,strcat(mdl,'/',blockName));

%Hide the name for supporting nodes.
set_param(blkHndl,'ShowName','off');
blkHeight = 65;
blkWidth = 110;
fusion.simulink.internal.trackingArchitecture.resizeBlock(blkHndl,blkHeight,blkWidth);
nodePos = get_param(nodeHndl,'position');
xOffset = 50;
yOffset = 10;
xpos = nodePos(1) - xOffset - blkWidth;
ypos = nodePos(2) - yOffset;
fusion.simulink.internal.trackingArchitecture.positionBlock(blkHndl,xpos,ypos);
set_param(blkHndl,'NumInputs',num2str(numSources));
fusion.simulink.internal.trackingArchitecture.connectBlocks(mdl, blkHndl, 1, nodeHndl, 1);
end
