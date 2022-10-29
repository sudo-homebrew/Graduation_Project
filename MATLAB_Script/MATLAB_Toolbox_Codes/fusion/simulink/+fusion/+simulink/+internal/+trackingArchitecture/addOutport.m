function addOutport(mdl,nodeHndl,summary,nodeIdx)
%   Add an outport and connect with the node.
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2021 The MathWorks, Inc.

ArchitectureOutputs = summary.ArchitectureOutput;
if iscell(ArchitectureOutputs)
    ArchOutputIndex = ArchitectureOutputs{nodeIdx};
elseif isscalar(ArchitectureOutputs)
    ArchOutputIndex = ArchitectureOutputs;
else
    ArchOutputIndex = ArchitectureOutputs(nodeIdx);
end
name = ['Tracks Out', num2str(ArchOutputIndex)];

h = add_block('simulink/Sinks/Out1',strcat(mdl,'/',name));
blkPos = get_param(nodeHndl,'position');
fusion.simulink.internal.trackingArchitecture.resizeBlock(h,15,20);
xpos = blkPos(3)+40;
yOffset = 40;
ypos = blkPos(2) + yOffset;
fusion.simulink.internal.trackingArchitecture.positionBlock(h,xpos,ypos);
fusion.simulink.internal.trackingArchitecture.connectBlocks(mdl,nodeHndl,1,h,1);
end