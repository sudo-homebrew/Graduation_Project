function blkhndl = addInport(mdl,nodeHndl,name)
%   Add an inport to the model and connect it with the node.
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2021 The MathWorks, Inc.

% Check if the input port is already existing in the model.
if ~any(matches(find_system(mdl,'BlockType','Inport'),strcat(mdl,'/',name)))
    blkhndl= add_block('simulink/Sources/In1',strcat(mdl,'/', name));
    fusion.simulink.internal.trackingArchitecture.resizeBlock(blkhndl,15,20);
end
fusion.simulink.internal.trackingArchitecture.connectBlocks(mdl, strcat(mdl,'/', name), 1, nodeHndl, NaN);
end