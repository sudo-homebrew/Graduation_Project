function hLine = connectBlocks(model, blk1, portInd1, blk2, portInd2)
%   Add a Signal between blk1/portInd1 and blk2/portInd2, and return the
%   line handle.
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2021 The MathWorks, Inc.

%Add a signal line between blk1 and blk2.
blk1Ports = get_param(blk1, 'PortHandles');
blk1OutputPort = blk1Ports.Outport(portInd1);
blk2Ports = get_param(blk2, 'PortHandles');
if isnan(portInd2) % Need to find an empty port
    for i = 1:numel(blk2Ports.Inport)
        p = get_param(blk2Ports.Inport(i),'Line');
        if p==-1 %Not connected
            blk2InputPort = blk2Ports.Inport(i);
            break
        end
    end
else
    blk2InputPort = blk2Ports.Inport(portInd2);
end
hLine = add_line(model,blk1OutputPort,blk2InputPort,'autorouting','on');
end