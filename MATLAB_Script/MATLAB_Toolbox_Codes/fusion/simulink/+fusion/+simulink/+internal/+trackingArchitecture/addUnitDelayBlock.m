function blkHndl = addUnitDelayBlock(mdl,nodeIdx,nodehndl,node)
%   Add a unit delay block in the model return the handle to the model. 
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2021 The MathWorks, Inc.

name =['F', num2str(nodeIdx),':Delay'];
blkHndl = add_block('simulink/Discrete/Unit Delay',strcat(mdl,'/',name));
set_param(blkHndl,'ShowName','off');
nodePos = get_param(nodehndl,'position'); %Connected blocks position
xpos = nodePos(3) + 30 +  node.YPosition * 5;
ypos = nodePos(4) - 15;
fusion.simulink.internal.trackingArchitecture.positionBlock(blkHndl,xpos,ypos);
%Connect node to delay block
fusion.simulink.internal.trackingArchitecture.connectBlocks(mdl,nodehndl,1,blkHndl,1);
end